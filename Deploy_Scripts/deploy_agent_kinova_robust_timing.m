%% deploy_agent_kinova_robust_timing.m  (V2.2 - robuste Zeitbasis + Timing-Diagnose)
% Deployt einen trainierten PPO-Agenten auf den echten Kinova Gen3 via
% kortexApiMexInterface (MEX). Repliziert die Simulink-Post-Action-Pipeline
% (Saturation -> Discrete Filter -> Rate Limiter) 1:1 aus dem Trainings-
% Modell und wendet darauf die Hardware-Schutzschicht an.
%
% AENDERUNGEN gegenueber V2.1:
%   - Referenzzeit ist jetzt robust: "guarded_wall" als Default.
%     Das verhindert sowohl zu kurze Solltrajektorien bei langsamer Loop
%     als auch Spruenge nach vorne durch blockierende MEX/getAction-Aufrufe.
%   - Agent/Feedback werden vor dem Start warmgelaufen, damit der erste
%     echte Regelzyklus nicht die Referenzzeit verschiebt.
%   - Logs enthalten jetzt t_wall, t_ref und dt_loop getrennt.
%   - XZ-Plot zeigt drei verschiedene Dinge getrennt:
%       1) kompletter geplanter Halbkreis
%       2) tatsaechlich waehrend des Laufs verwendete Sollpunkte
%       3) Isttrajektorie aus FK/Kortex.
%   - Timing-Watchdog stoppt die Hardware, wenn die Loop zu lange haengt.
%
% SICHERHEITSKONZEPT (unveraendert):
%   - Dry-Run Modus
%   - Pre-Flight Gate
%   - 29D Obs-Konsistenz-Check
%   - Trainings-Pipeline-Replikation (unskaliert)
%   - HW-Hard-Cap als separate Schutzschicht
%   - Soft-Limit-Bremsung
%   - Fault-Check, OOD-Stopp bei ||ep|| > 0.4 m
%   - onCleanup: Zero-Velocity + DestroyRobotApisWrapper

clear; clc; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();

% ---- SICHERHEIT: Dry-Run + Pre-Flight ----
cfg.dryRun            = false;
cfg.preFlightRequired = false;

% ---- Lauf-Identifikation (fuer run_logger / C3) ----
cfg.seed = 3;   % manuell pro Lauf hochzaehlen: 1, 2, 3, ...

% ---- Agent laden ----
cfg.agentFile      = "";
cfg.expectedObsDim = 29;

% ---- URDF ----
cfg.urdfFile   = "../SpaceKinova.urdf";
cfg.eeBodyName = "kinova_end_effector_link";
cfg.toolOffset = [0; 0; 0];

% ---- MEX-Interface ----
cfg.kinovaIP         = '192.168.0.10';
cfg.kinovaUser       = 'admin';
cfg.kinovaPassword   = 'admin';
cfg.sessionTimeoutMs = uint32(60000);
cfg.controlTimeoutMs = uint32(200);
cfg.speedCmdDuration = 0;

% ---- Kinova Gen3 Spezifikationen (rad, rad/s) ----
cfg.nJ         = 7;
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
cfg.dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];

% ---- HW-Safety-Cap ----
cfg.safetyFactor = 0.75;
cfg.dqHwCap      = cfg.safetyFactor * cfg.dqLim;

% ---- Uniform Speed-Scale ----
cfg.speedScale = 0.3;

% ---- Trainings-Pipeline-Parameter ----
cfg.satUpper     = [0.0; 0.9774; 0.0; 0.9774; 0.0; 0.1; 0.0];
cfg.satLower     = -cfg.satUpper;
cfg.filt_num     = 0.05;
cfg.filt_den     = 0.95;
cfg.slewRate     = 0.5;

% ---- HW-Schutz ----
cfg.qSoftMargin  = deg2rad(10);

% ---- Steuerung ----
cfg.rateHz          = 40;
cfg.Ts              = 1/cfg.rateHz;
cfg.maxDuration     = 8.5;
cfg.watchdogTimeout = 0.10;

% ---- Referenz-Zeitbasis ----
% "guarded_wall" ist fuer echte Hardware der sicherste Default:
%   - normale 40-Hz-Loop: Referenz folgt echter Zeit
%   - kurzer Blocker/JIT/MEX-Stall: Referenz springt NICHT weit nach vorne
%   - dauerhafte langsame Loop: Watchdog stoppt statt falsche Trajektorie zu loggen
% Weitere Modi nur fuer Debugging:
%   "wall"   : reine toc()-Zeit, kann bei Blockern zu weit springen
%   "sample" : t=(k-1)*Ts, kann bei langsamer Loop zu kurz wirken
cfg.referenceTiming      = "guarded_wall";
cfg.maxRefAdvanceFactor = 1.25;       % max. Referenzfortschritt pro Step = Faktor*Ts
cfg.stopOnTimingOverrun = true;       % bei dt_loop > watchdogTimeout sofort stoppen
cfg.rateControlInDryRun = true;       % Dry-Run ebenfalls in Echtzeit takten

% ---- Homing ----
cfg.homingSpeed       = deg2rad(10);
cfg.homingTolerance   = 0.02;
cfg.homingWaitTimeout = 20;
cfg.homingPosTol      = deg2rad(1.0);

% ---- Referenztrajektorie (URDF/Training-Frame) ----
cfg.r      = 0.2;
cfg.center = [0.0, -0.025, 1.687 - cfg.r];
cfg.omega  = pi/cfg.maxDuration;
cfg.yConst = 0;

% ---- Realer Base-Offset ----
cfg.realBaseOffset = [0 0 0.001];

% ---- Observation Limits ----
cfg.ePLim   = 0.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

cfg.oodThreshold     = 0.4;
cfg.divergenceWarn   = 0.2;   % Eskalations-Warnung vor OOD

%% =========================
%  1) PRE-FLIGHT-GATE
%  =========================
if ~cfg.dryRun && cfg.preFlightRequired
    error(['Pre-Flight fehlt. Bitte erst in Simulink mit identischer ' ...
           'Observation-Struktur + 40 Hz laufen lassen und Return/KPIs ' ...
           'pruefen. Danach cfg.preFlightRequired = false setzen.']);
end

%% =========================
%  2) AGENT LADEN + OBS-KONSISTENZ
%  =========================
if strlength(cfg.agentFile) == 0
    d = dir(fullfile("../SavedAgents/MotionProfile/CDR/PPO/CDR2-4.mat"));
    if isempty(d)
        error("Kein trainierter Agent gefunden in savedAgents_spacekinova/");
    end
    [~, idx] = max([d.datenum]);
    cfg.agentFile = fullfile(d(idx).folder, d(idx).name);
end

fprintf("Lade Agent: %s\n", cfg.agentFile);
loaded = load(cfg.agentFile);
agent  = loaded.agent;

agent.UseExplorationPolicy = false;

obsInfo      = getObservationInfo(agent);
actualObsDim = obsInfo.Dimension(1);
assert(actualObsDim == cfg.expectedObsDim, ...
    "ObservationDim Mismatch: Agent=%d, Deploy=%d.", ...
    actualObsDim, cfg.expectedObsDim);

%% =========================
%  3) ROBOT-MODELL LADEN
%  =========================
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

%% =========================
%  4) REFERENZTRAJEKTORIE / PLAN FUER PLOTS UND PREFLIGHT
%  =========================
% Wichtig: Dieser t_ref_full ist NUR der komplette Plan fuer Preflight und
% Plot. In der Hardware-Loop wird die Referenz aus log.t_ref berechnet,
% nicht ueber den Schleifenindex.
t_ref_full = linspace(0, cfg.maxDuration, max(2, ceil(cfg.maxDuration * cfg.rateHz) + 1));
[traj_ref, vref_full] = referenceTrajectory(t_ref_full, cfg); %#ok<NASGU>

%% =========================
%  5) OBSERVATION LIMITS
%  =========================
obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
    cfg.qLim_lower; ...
    -cfg.dqLim; ...
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ];
obsHigh = [ ...
    cfg.ePLim   * ones(3,1); ...
    cfg.eVLim   * ones(3,1); ...
    cfg.qLim_upper; ...
    cfg.dqLim; ...
    cfg.vBLim   * ones(3,1); ...
    cfg.wBLim   * ones(3,1); ...
    cfg.eOriLim * ones(3,1) ];

%% =========================
%  5b) REACHABILITY-PREFLIGHT
%  =========================
fprintf("\n== Reachability-Preflight ==\n");
ikCheck     = inverseKinematics('RigidBodyTree', robot_rbt);
weightsChk  = [0.25 0.25 0.25 1 1 1];
nSamples    = 20;
sampleIdx   = round(linspace(1, size(traj_ref, 1), nSamples));
q_seed      = zeros(1, cfg.nJ);
q_prev      = [];
maxJump_rad = deg2rad(45);

qMin = +inf(1, cfg.nJ);
qMax = -inf(1, cfg.nJ);
reachabilityOk = true;

for s = 1:nSamples
    idx     = sampleIdx(s);
    T_chk   = trvec2tform(traj_ref(idx, :));
    [q_s, info] = ikCheck(char(cfg.eeBodyName), T_chk, weightsChk, q_seed);
    if info.ExitFlag <= 0
        fprintf("  [WARN] IK konvergierte nicht bei idx=%d (t=%.2fs)\n", ...
                idx, t_ref_full(idx));
        reachabilityOk = false;
    end
    qMin = min(qMin, q_s);
    qMax = max(qMax, q_s);

    viol = (q_s(:) < cfg.qLim_lower) | (q_s(:) > cfg.qLim_upper);
    if any(viol)
        fprintf("  [WARN] Joint-Limit-Verletzung bei idx=%d: J%s\n", ...
                idx, mat2str(find(viol)));
        reachabilityOk = false;
    end

    if ~isempty(q_prev)
        jump = wrapPi(q_s(:) - q_prev(:));
        if any(abs(jump) > maxJump_rad)
            fprintf("  [WARN] Grosser Joint-Sprung bei idx=%d: max=%.1f deg \n", ...
                idx, rad2deg(max(abs(jump))));
            reachabilityOk = false;
        end
    end
    q_seed = q_s;
    q_prev = q_s;
end

fprintf("  Joint-Range ueber Trajektorie [deg]:\n");
for j = 1:cfg.nJ
    fprintf("    J%d: [%+7.2f, %+7.2f]\n", j, ...
            rad2deg(qMin(j)), rad2deg(qMax(j)));
end

if ~reachabilityOk
    warning(['Reachability-Preflight hat Auffaelligkeiten. Trajektorie ' ...
             'und URDF pruefen, bevor auf echter HW gestartet wird.']);
end

if any(cfg.realBaseOffset ~= 0)
    p_urdf_start  = traj_ref(1, :);
    p_urdf_mid    = traj_ref(round(end/2), :);
    p_urdf_end    = traj_ref(end, :);
    p_world_start = p_urdf_start + cfg.realBaseOffset;
    p_world_mid   = p_urdf_mid   + cfg.realBaseOffset;
    p_world_end   = p_urdf_end   + cfg.realBaseOffset;
    fprintf("\n  Trajektorien-Eckpunkte (Welt-Frame, fuer Arbeitsraum-Check):\n");
    fprintf("    Start:  [%+.3f %+.3f %+.3f] m\n", p_world_start);
    fprintf("    Mitte:  [%+.3f %+.3f %+.3f] m\n", p_world_mid);
    fprintf("    Ende:   [%+.3f %+.3f %+.3f] m\n", p_world_end);
    fprintf("    (Kreis mit r=%.2f m in XZ, y-konstant)\n", cfg.r);
else
    fprintf(["\n  Hinweis: cfg.realBaseOffset = [0 0 0] -- keine " ...
             "Welt-Frame-Anzeige.\n"]);
end
fprintf("\n");

%% =========================
%  6) MEX-VERBINDUNG + DIAGNOSE
%  =========================
apiHandle = [];

if ~cfg.dryRun
    fprintf("== MEX init ==\n");
    apiHandle = kinovaOpen(cfg);

    cleanupObj = onCleanup(@() safeShutdown(apiHandle, cfg.nJ));  %#ok<NASGU>

    fprintf("Warte auf erstes Feedback (Diagnose)...\n");
    [err, baseFb, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
    if err ~= 0
        error("RefreshFeedback fehlgeschlagen (errorCode=%d).", err);
    end
    fprintf("   actuatorFb: numel=%d, fields = {%s}\n", ...
            numel(actuatorFb), strjoin(fieldnames(actuatorFb), ', '));
    if isfield(actuatorFb, 'position')
        fprintf("   actuatorFb(1).position = %.3f (erwartet in GRAD)\n", ...
                double(actuatorFb(1).position));
    end
    if isfield(actuatorFb, 'velocity')
        fprintf("   actuatorFb(1).velocity = %.3f (erwartet in deg/s)\n", ...
                double(actuatorFb(1).velocity));
    end
    if ~isempty(baseFb)
        fprintf("   baseFb fields = {%s}\n", strjoin(fieldnames(baseFb), ', '));
    end

    [q0, dq0, ~] = kinovaReadFeedback(apiHandle, cfg);
    fprintf("Verbindung OK. q0 [rad] = [%s]\n", ...
            join(string(round(q0, 3)), ", "));
else
    fprintf("\n=== DRY-RUN MODUS ===\n");
    fprintf("Keine MEX-Verbindung. Kommandos werden nur angezeigt.\n\n");
    q0  = zeros(cfg.nJ, 1);
    dq0 = zeros(cfg.nJ, 1);
end

%% =========================
%  7) STARTPOSITION UEBERNEHMEN
%  =========================
if ~cfg.dryRun
    [q_now, ~, ~] = kinovaReadFeedback(apiHandle, cfg);
else
    q_now = q0;
end

T_ee_now  = getTransform(robot_rbt, q_now.', char(cfg.eeBodyName));
err_start = norm(T_ee_now(1:3, 4) - traj_ref(1, :)');

if err_start < 0.05
    fprintf("Startpose uebernommen. Abweichung = %.4f m\n", err_start);
else
    warning("Manuelle Startpose weicht %.3f m von Referenz ab. Tracking wird mit Offset starten.", err_start);
end

%% =========================
%  8) SICHERHEITSHINWEIS
%  =========================
fprintf("\n== SICHERHEITSHINWEIS ==\n");
fprintf(" - Arbeitsraum frei, E-Stop bereit\n");
fprintf(" - Agent-Pipeline: Trainings-identisch (kein Gain, sat=[%s])\n", ...
        join(string(cfg.satUpper), " "));
fprintf(" - HW-Hard-Cap: safetyFactor=%.2f -> dqHwCap=[%s] rad/s\n", ...
        cfg.safetyFactor, join(string(round(cfg.dqHwCap, 3)), ", "));
fprintf(" - Speed-Scale (uniform, nach HW-Cap): %.2f\n", cfg.speedScale);
fprintf(" - Deterministische Policy (UseExplorationPolicy=false)\n");
fprintf(" - Laufzeit: %.1f s\n", cfg.maxDuration);
fprintf(" - Trajektorie wird im URDF-Frame gerechnet (= Training-Frame).\n");
fprintf("   Der reale EE faehrt physisch um cfg.realBaseOffset = [%s] verschoben.\n", ...
        join(string(cfg.realBaseOffset), " "));
fprintf(" - Lauf-Seed: %d (fuer run_logger Label)\n", cfg.seed);
fprintf(" - Dry-Run: %s\n\n", string(cfg.dryRun));

fprintf("== Cap-Analyse (Training-Sat vs. HW-Cap) ==\n");
fprintf(" Joint | satUpper | dqHwCap | Cap biest? | max dq_cmd nach scale\n");
fprintf(" ------|----------|---------|------------|-----------------------\n");
anyBites = false;
for j = 1:cfg.nJ
    sat_j  = cfg.satUpper(j);
    cap_j  = cfg.dqHwCap(j);
    eff_j  = min(sat_j, cap_j);
    bites  = (sat_j > 0) && (cap_j < sat_j - 1e-6);
    if bites, anyBites = true; end
    max_cmd = cfg.speedScale * eff_j;
    fprintf("  J%d   |  %5.3f  |  %5.3f  |     %s    |  %5.3f rad/s (%5.2f deg/s)\n", ...
        j, sat_j, cap_j, ...
        ternary(bites, "JA", "nein"), max_cmd, rad2deg(max_cmd));
end
if anyBites
    warning(["HW-Cap beisst auf aktiven Joints -> Training-Ratios werden " ...
             "verzerrt. Erhoehe cfg.safetyFactor oder reduziere stattdessen " ...
             "cfg.speedScale, um sicher langsamer zu fahren."]);
else
    fprintf("  OK: HW-Cap inaktiv auf allen aktiven Joints -> Ratios bleiben erhalten.\n");
end
fprintf("\n");

if ~cfg.dryRun
    input("ENTER zum Starten (CTRL+C zum Abbrechen)... ", "s");
end

%% =========================
%  8b) WARM-UP VOR DEM ECHTEN ZEITSTART
%  =========================
% Verhindert, dass JIT, erstes getAction() oder ein langsames erstes
% RefreshFeedback die Referenzzeit nach vorne ziehen, bevor der Roboter
% ueberhaupt einen ersten Speed-Command erhalten hat.
fprintf("== Warm-up vor Start ==\n");
dummyObs = zeros(cfg.expectedObsDim, 1);
try
    dummyAction = getAction(agent, {dummyObs}); %#ok<NASGU>
    fprintf("  getAction warm-up OK\n");
catch ME
    warning("getAction warm-up fehlgeschlagen: %s", ME.message);
end

if ~cfg.dryRun
    for ii = 1:3
        [q_now, dq0, faultFlagWarm, toolPoseWorldWarm] = kinovaReadFeedback(apiHandle, cfg); %#ok<ASGLU>
        if faultFlagWarm
            error('Fault waehrend Warm-up -- Abbruch.');
        end
        pause(0.02);
    end
    fprintf("  Feedback warm-up OK\n");
else
    toolPoseWorldWarm = [NaN NaN NaN]; %#ok<NASGU>
end
fprintf("  Timing wird erst JETZT im Loop gestartet.\n\n");

%% =========================
%  9) PIPELINE-STATE
%  =========================
state = struct();
state.yFilt  = zeros(cfg.nJ, 1);
state.dqPrev = zeros(cfg.nJ, 1);

%% =========================
%  10) HAUPTSCHLEIFE
%  =========================
% Wir reservieren etwas mehr Speicher als ideal noetig. Beendet wird ueber
% t_ref >= cfg.maxDuration oder Sicherheitsstopps.
nStepsPlan = ceil(cfg.maxDuration * cfg.rateHz) + 1;
nStepsMax  = nStepsPlan + ceil(2.0 * cfg.rateHz);   % kleiner Puffer fuer Stop/Diagnose
log = initLog(nStepsMax, cfg.nJ);

q  = q_now;
dq = dq0;
warned_divergence = false;   % flag fuer Eskalations-Warnung

fprintf("== Starte Deployment-Loop: mode=%s, %.1f Hz, max %.2f s ==\n", ...
        cfg.referenceTiming, cfg.rateHz, cfg.maxDuration);
fprintf("   Watchdog: dt_loop > %.3f s -> %s\n", cfg.watchdogTimeout, ...
        ternary(cfg.stopOnTimingOverrun, "STOPP", "nur Warnung"));
fprintf("   guarded_wall: max dt_ref/step = %.4f s\n\n", ...
        cfg.maxRefAdvanceFactor * cfg.Ts);

r = rateControl(cfg.rateHz);
try
    reset(r);
catch
    % Aeltere MATLAB-Versionen: rateControl startet beim Erzeugen.
end

k_end = 0;
tRunStart = tic;
tRef = 0.0;
tWallPrevLoop = 0.0;
tLastSendWall = NaN;

for k = 1:nStepsMax
    tWallLoop = toc(tRunStart);

    if k == 1
        dtLoop = 0.0;
    else
        dtLoop = tWallLoop - tWallPrevLoop;
    end
    tWallPrevLoop = tWallLoop;

    % --- Harte Timing-Sicherheit: falls NACH einem gesendeten Kommando
    %     zu lange nichts Neues gesendet wurde, zuerst Null senden und abbrechen.
    %     Wichtig: Das misst t seit letztem Send, nicht nur loop-start-to-start.
    if isnan(tLastSendWall)
        dtSinceSend = NaN;
    else
        dtSinceSend = tWallLoop - tLastSendWall;
    end

    timingOverrun = ~isnan(dtSinceSend) && (dtSinceSend > cfg.watchdogTimeout);
    if timingOverrun
        fprintf(['\n[TIMING-STOPP k=%d] dt_since_send=%.4f s > watchdog %.4f s. ' ...
                 'Sende Zero-Velocity und breche ab.\n'], ...
                 k, dtSinceSend, cfg.watchdogTimeout);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        if cfg.stopOnTimingOverrun
            break;
        end
    end

    % --- Referenz-Zeit bestimmen ---
    switch string(cfg.referenceTiming)
        case "wall"
            % Reine Echtzeit. Gut nur, wenn kein JIT/MEX-Blocker auftritt.
            tRef = min(tWallLoop, cfg.maxDuration);
        case "sample"
            % Trainings-/Sample-Zeit. Gut fuer Debug, aber falsch bei langsamer HW-Loop.
            tRef = min((k - 1) * cfg.Ts, cfg.maxDuration);
        case "guarded_wall"
            % Echtzeit, aber mit maximalem Fortschritt pro Iteration.
            % Dadurch kann ein einzelner Blocker die Solltrajektorie nicht
            % mehrere Sekunden nach vorne katapultieren.
            if k == 1
                tRef = 0.0;
            else
                maxRefStep = cfg.maxRefAdvanceFactor * cfg.Ts;
                dtRef = min(max(dtLoop, 0.0), maxRefStep);
                tRef = min(tRef + dtRef, cfg.maxDuration);
            end
        otherwise
            error('Unbekanntes cfg.referenceTiming: %s', cfg.referenceTiming);
    end

    % Nach Ende der Referenz noch einen finalen Null-Speed senden und beenden.
    if tRef >= cfg.maxDuration && k > 1
        fprintf("\n[INFO] Referenzende erreicht: t_ref=%.3f s\n", tRef);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        break;
    end

    % --- Zustand lesen + Fault-Check ---
    tBeforeFeedback = toc(tRunStart);
    if ~cfg.dryRun
        [q, dq, faultFlag, toolPoseWorld] = kinovaReadFeedback(apiHandle, cfg);
        if faultFlag
            fprintf('\n[FAULT] Kinova-Fault erkannt -- Stopp\n');
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
            k_end = k;
            break;
        end
    else
        toolPoseWorld = [NaN NaN NaN];
    end
    tAfterFeedback = toc(tRunStart);

    % --- FK + Jacobian ---
    T_ee   = getTransform(robot_rbt, q.', char(cfg.eeBodyName));
    ee_pos = T_ee(1:3, 4);
    ee_rot = T_ee(1:3, 1:3);
    J_ee   = geometricJacobian(robot_rbt, q.', char(cfg.eeBodyName));

    ee_pos_tool = ee_pos + ee_rot * cfg.toolOffset(:);

    % --- Referenz + Fehler ---
    [ref_pos_row, ref_vel_row] = referenceTrajectory(tRef, cfg);
    ref_pos = ref_pos_row.';
    ref_vel = ref_vel_row.';

    ep      = ref_pos - ee_pos;
    ee_vel  = J_ee(4:6, :) * dq;
    ev      = ref_vel - ee_vel;

    R_err = eye(3) * ee_rot.';
    e_ori = [R_err(3,2) - R_err(2,3); ...
             R_err(1,3) - R_err(3,1); ...
             R_err(2,1) - R_err(1,2)] / 2;

    v_base = zeros(3, 1);
    w_base = zeros(3, 1);

    % --- Observation (29D) ---
    obs = [ep; ev; q; dq; v_base; w_base; e_ori];

    % --- Eskalations-Warnung vor OOD-Stopp ---
    if norm(ep) > cfg.divergenceWarn && ~warned_divergence
        fprintf('\n[WARN k=%d] ||ep||=%.3f m > %.2f m -- Divergenz erkannt, beobachte weiter...\n', ...
                k, norm(ep), cfg.divergenceWarn);
        warned_divergence = true;
    end

    % --- OOD-Check (harter Stopp) ---
    if norm(ep) > cfg.oodThreshold
        fprintf(['\n[STOPP k=%d] Positionsfehler %.3f m > Schwelle %.3f m ' ...
                 '(Out-of-Distribution)\n'], k, norm(ep), cfg.oodThreshold);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        k_end = k;
        break;
    end

    % --- Obs-Clipping ---
    obs = max(min(obs, obsHigh), obsLow);

    % --- Agent abfragen ---
    tBeforeAgent = toc(tRunStart);
    action = getAction(agent, {obs});
    tAfterAgent = toc(tRunStart);
    dq_raw = action{1}(:);

    % --- Defensives Clipping auf [-1, 1] ---
    dq_raw = max(min(dq_raw, 1), -1);

    % --- Pipeline (Sat -> Filter -> RateLim) ---
    [dq_cmd, state] = applyTrainingPipeline(dq_raw, state, cfg);

    % --- HW-Schutz: Soft-Limit-Bremsung ---
    for j = 1:cfg.nJ
        distToUpper = cfg.qLim_upper(j) - q(j);
        distToLower = q(j) - cfg.qLim_lower(j);
        if distToUpper < cfg.qSoftMargin && dq_cmd(j) > 0
            dq_cmd(j) = dq_cmd(j) * max(distToUpper / cfg.qSoftMargin, 0);
        end
        if distToLower < cfg.qSoftMargin && dq_cmd(j) < 0
            dq_cmd(j) = dq_cmd(j) * max(distToLower / cfg.qSoftMargin, 0);
        end
    end

    % --- HW-Schutz: Hard-Cap ---
    dq_cmd = max(min(dq_cmd, cfg.dqHwCap), -cfg.dqHwCap);

    % --- Uniform Speed-Scale ---
    dq_cmd = cfg.speedScale * dq_cmd;

    % --- MULTI-STEP-DUMP (nach Pipeline + Safety) ---
    if any(k == [1, 2, 3, 5, 10, 20, 50, 100])
        fprintf('\n=== STEP %d DUMP (t_wall=%.3fs, t_ref=%.3fs, dt=%.4fs) ===\n', ...
                k, tWallLoop, tRef, dtLoop);
        fprintf('  Feedback-Zeit = %.4f s, Agent-Zeit = %.4f s\n', ...
                tAfterFeedback - tBeforeFeedback, tAfterAgent - tBeforeAgent);
        fprintf('  q       = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad\n', q);
        fprintf('  ep      = [%+.4f %+.4f %+.4f] m, ||ep||=%.4f\n', ep, norm(ep));
        fprintf('  ev      = [%+.4f %+.4f %+.4f] m/s, ||ev||=%.4f\n', ev, norm(ev));
        fprintf('  dq_raw  = [%+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f]\n', dq_raw);
        fprintf('  yFilt   = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f]\n', state.yFilt);
        fprintf('  dq_cmd  = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad/s\n', dq_cmd);
        fprintf('  dq_meas = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad/s\n', dq);
        fprintf('=========================\n');
    end

    % --- Senden / Anzeigen ---
    if cfg.dryRun
        if mod(k, cfg.rateHz) == 1
            fprintf("t_wall=%.2fs | t_ref=%.2fs | ep=[%.3f %.3f %.3f]m | dq_cmd=[%s] rad/s\n", ...
                tWallLoop, tRef, ep(1), ep(2), ep(3), ...
                join(string(round(dq_cmd, 4)), " "));
        end
    else
        err = kinovaSendVelocity(apiHandle, dq_cmd, cfg);
        if err ~= 0
            warning('SendJointSpeedCommand errorCode=%d -- Stopp.', err);
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
            k_end = k;
            break;
        end
    end

    tAfterSend = toc(tRunStart);
    tLastSendWall = tAfterSend;

    % --- Logging ---
    log.t(k)              = tRef;       % Kompatibilitaet: t == Referenzzeit
    log.t_ref(k)          = tRef;
    log.t_wall(k)         = tWallLoop;
    log.dt_loop(k)        = dtLoop;
    log.dt_since_send(k)  = dtSinceSend;
    log.t_feedback(k)     = tAfterFeedback - tBeforeFeedback;
    log.t_agent(k)        = tAfterAgent - tBeforeAgent;
    log.t_send(k)         = tAfterSend - tAfterAgent;
    log.timing_overrun(k) = timingOverrun;
    log.q(k, :)           = q.';
    log.dq(k, :)          = dq.';
    log.dq_raw(k, :)      = dq_raw.';
    log.dq_cmd(k, :)      = dq_cmd.';
    log.dq_filt(k, :)     = state.yFilt.';
    log.yFilt(k, :)       = state.yFilt.';
    log.ee_pos(k, :)      = ee_pos.';
    log.ee_tool(k, :)     = ee_pos_tool.';
    log.ee_ref(k, :)      = ref_pos.';
    log.ep(k, :)          = ep.';
    log.ep_norm(k)        = norm(ep);
    log.ee_kortex(k, :)   = toolPoseWorld - cfg.realBaseOffset;

    k_end = k;

    % --- Takt halten ---
    if ~cfg.dryRun || cfg.rateControlInDryRun
        waitfor(r);
    end
end

if ~cfg.dryRun
    kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
end

fprintf("\n== Deployment beendet (k=%d/%d) ==\n", k_end, nStepsMax);
%% =========================
%  11) LOG + AUSWERTUNG
%  =========================

% Truncate ausschliesslich ueber k_end aus der Loop. Nicht aus ee_ref
% erraten, denn Referenzpunkte koennen legitimerweise Nullen enthalten.
if k_end < 1
    warning('Kein gueltiger Log-Step vorhanden. Setze k_end=1 fuer Diagnose.');
    k_end = 1;
end
log = truncateLog(log, k_end);

ep_norms = vecnorm(log.ep, 2, 2);
fprintf("Position Tracking RMSE: %.4f m\n", sqrt(mean(ep_norms.^2)));
fprintf("Position Tracking Max:  %.4f m\n", max(ep_norms));

if ~any(isnan(log.ee_kortex(:)))
    delta_fk_kortex = vecnorm(log.ee_pos - log.ee_kortex, 2, 2);
    fprintf("\n== FK vs Kortex (URDF-Konsistenz-Check) ==\n");
    fprintf("  Start-Discrepancy : %.4f m\n", delta_fk_kortex(1));
    fprintf("  End-Discrepancy   : %.4f m\n", delta_fk_kortex(end));
    fprintf("  Max-Discrepancy   : %.4f m\n", max(delta_fk_kortex));
    if max(delta_fk_kortex) > 0.02
        fprintf(['  --> SIGNIFIKANTE DIVERGENZ: URDF passt nicht zum ' ...
                 'realen Gen3. Agent-Training basiert auf einer anderen\n' ...
                 '      Kinematik als die HW ausfuehrt. Siehe xz-Plot ' ...
                 '(Kortex-Kurve = Wahrheit, FK-Kurve = Agent-Annahme).\n']);
    else
        fprintf("  --> OK: URDF konsistent mit realem Gen3.\n");
    end
end

fprintf("\n== Pipeline-Diagnose (max |.| pro Joint) ==\n");
dq_raw_max = max(abs(log.dq_raw), [], 1);
dq_cmd_max = max(abs(log.dq_cmd), [], 1);
fprintf("  dq_raw  [-1..1]   : %s\n", mat2str(round(dq_raw_max, 3)));
fprintf("  dq_cmd  [rad/s]   : %s\n", mat2str(round(dq_cmd_max, 3)));
fprintf("  satUpper[rad/s]   : %s\n", mat2str(round(cfg.satUpper.', 3)));
fprintf("  dqHwCap [rad/s]   : %s\n", mat2str(round(cfg.dqHwCap.', 3)));
fprintf("  speedScale        : %.3f\n", cfg.speedScale);
fprintf("\n  Pro Joint: wer limitiert?\n");
for j = 1:cfg.nJ
    if cfg.satUpper(j) == 0
        continue;
    end
    expected_sat_scaled = cfg.speedScale * cfg.satUpper(j);
    expected_cap_scaled = cfg.speedScale * cfg.dqHwCap(j);
    actual              = dq_cmd_max(j);
    if abs(actual - expected_sat_scaled) < 1e-3
        dominant = "Training-Saturation";
    elseif abs(actual - expected_cap_scaled) < 1e-3
        dominant = "HW-Cap  <-- Ratio-Verzerrung!";
    else
        dominant = "keiner (Agent hat Limit nicht erreicht)";
    end
    fprintf("    J%d: actual=%.3f, sat*scale=%.3f, cap*scale=%.3f -> %s\n", ...
        j, actual, expected_sat_scaled, expected_cap_scaled, dominant);
end
fprintf("\n");

% --- Speichern: alter Diagnose-Stil (cfg + log) ---
logFile = sprintf("deploy_log_%s.mat", ...
                  string(datetime('now'), 'yyyyMMdd_HHmmss'));
save(logFile, "cfg", "log");
fprintf("Log gespeichert: %s\n", logFile);

% --- Speichern: run_logger-Format fuer C3 sim2real-Vergleich ---
if exist('run_logger', 'file') == 2
    data = struct();
    data.t           = log.t;          % Referenzzeit, kompatibel zu altem Schema
    data.t_ref       = log.t_ref;
    data.t_wall      = log.t_wall;
    data.dt_loop     = log.dt_loop;
    data.dt_since_send = log.dt_since_send;
    data.q_measured  = rad2deg(log.q);
    data.dq_cmd      = rad2deg(log.dq_cmd);
    data.dq_measured = rad2deg(log.dq);
    data.dq_raw      = log.dq_raw;
    data.dq_filt     = log.dq_filt;
    data.ee_measured = log.ee_pos;
    data.ee_kortex   = log.ee_kortex;
    data.ee_ref      = log.ee_ref;
    data.ep          = log.ep;
    data.ep_norm     = log.ep_norm;

    meta = struct();
    meta.label         = sprintf('agent_train_seed%d', cfg.seed);
    meta.source        = 'real';
    meta.startpose     = rad2deg(q_now)';
    meta.startposeName = 'training';
    meta.seed          = cfg.seed;
    meta.urdfFile      = cfg.urdfFile;
    meta.agentFile     = cfg.agentFile;
    meta.speedScale    = cfg.speedScale;
    meta.safetyFactor  = cfg.safetyFactor;
    meta.rateHz        = cfg.rateHz;
    meta.maxDuration   = cfg.maxDuration;
    meta.referenceTiming = cfg.referenceTiming;
    meta.dryRun        = cfg.dryRun;
    meta.kEnd          = k_end;
    meta.rmsEpM        = sqrt(mean(ep_norms.^2));
    meta.maxEpM        = max(ep_norms);
    meta.comment       = 'Agent-Deploy mit Trainings-Pipeline';

    run_logger(data, meta);
else
    fprintf("\n[Hinweis] run_logger.m nicht im Pfad - C3-Log uebersprungen.\n");
end

% =============== Plots ===============
figure('Name', 'Deployment: EE Tracking');
subplot(2,1,1);
plot(log.t_wall, log.ee_ref, '--', log.t_wall, log.ee_pos, '-');
legend('ref_x','ref_y','ref_z','x','y','z');
xlabel('t_{wall} [s]'); ylabel('Position [m]');
title('End-Effector Tracking: geloggte Referenz vs. FK'); grid on;
subplot(2,1,2);
plot(log.t_wall, ep_norms);
xlabel('t_{wall} [s]'); ylabel('||ep|| [m]'); title('Positionsfehler'); grid on;

figure('Name', 'Deployment: Timing-Diagnose');
subplot(3,1,1);
plot(log.t_wall, log.t_ref, 'LineWidth', 1.2); hold on;
plot(log.t_wall, log.t_wall, ':');
xlabel('t_{wall} [s]'); ylabel('t_{ref} [s]');
legend('verwendete Referenzzeit', 'Ideallinie t_{ref}=t_{wall}', 'Location', 'best');
title(sprintf('Zeitbasis: %s', cfg.referenceTiming)); grid on;
subplot(3,1,2);
plot(log.t_wall, log.dt_loop, 'LineWidth', 1.2); hold on;
plot(log.t_wall, log.dt_since_send, '--', 'LineWidth', 1.0);
yline(cfg.Ts, 'k:');
yline(cfg.watchdogTimeout, 'r--');
xlabel('t_{wall} [s]'); ylabel('Zeit [s]');
legend('dt_{loop}', 'dt seit letztem Send', 'Soll-Ts', 'Watchdog', 'Location', 'best'); grid on;
subplot(3,1,3);
plot(log.t_wall, log.t_feedback, '-', log.t_wall, log.t_agent, '--');
xlabel('t_{wall} [s]'); ylabel('Dauer [s]');
legend('RefreshFeedback', 'getAction', 'Location', 'best');
title('Wo Zeit verloren geht'); grid on;

figure('Name', 'Deployment: Pipeline-Wirkung');
subplot(3,1,1);
plot(log.t_wall, log.dq_raw);
yline([-1 1], 'r--');
xlabel('t_{wall} [s]'); ylabel('dq_{raw}'); title('Agent-Output (Pipeline-Input)');
legend(arrayfun(@(j) sprintf("J%d", j), 1:cfg.nJ, 'UniformOutput', false));
grid on;
subplot(3,1,2);
plot(log.t_wall, log.yFilt);
xlabel('t_{wall} [s]'); ylabel('y_{filt} [rad/s]');
title('Filter-Zustand (nach Sat + IIR-Filter)');
grid on;
subplot(3,1,3);
plot(log.t_wall, log.dq_cmd);
xlabel('t_{wall} [s]'); ylabel('dq_{cmd} [rad/s]');
title('Nach Rate-Limiter + HW-Safety (an Roboter gesendet)');
grid on;

figure('Name', 'Deployment: EE Trajektorie Ist vs. Soll (xz)');
plot(traj_ref(:,1), traj_ref(:,3), 'k:', 'LineWidth', 1.0); hold on;
plot(log.ee_ref(:,1), log.ee_ref(:,3), 'r--', 'LineWidth', 1.5);
plot(log.ee_pos(:,1), log.ee_pos(:,3), 'b--', 'LineWidth', 1.3);
if ~any(isnan(log.ee_kortex(:)))
    plot(log.ee_kortex(:,1), log.ee_kortex(:,3), 'm-',  'LineWidth', 1.8);
end
plot(log.ee_ref(1,1), log.ee_ref(1,3), 'ks', 'MarkerFaceColor','k');
plot(log.ee_pos(1,1), log.ee_pos(1,3), 'bo', 'MarkerFaceColor','b');
if ~any(isnan(log.ee_kortex(:)))
    plot(log.ee_kortex(1,1), log.ee_kortex(1,3), 'mo', 'MarkerFaceColor','m');
end
grid on; axis equal;
xlabel('x [m]'); ylabel('z [m]');
if ~any(isnan(log.ee_kortex(:)))
    legend('Soll komplett (geplanter Halbkreis)', ...
           'Soll verwendet/logged', ...
           'FK-Prediction (Agent sieht)', ...
           'Kortex-Wahrheit (echter Robot)', ...
           'Start Soll', 'Start FK', 'Start Kortex', ...
           'Location', 'best');
else
    legend('Soll komplett (geplanter Halbkreis)', ...
           'Soll verwendet/logged', ...
           'FK-Prediction', ...
           'Start Soll', 'Start FK', ...
           'Location', 'best');
end
title('End-Effector Trajektorie in xz-Ebene (URDF-Frame)');

if numel(log.t_wall) > 1
    dt_real = log.dt_loop(2:end);
    fprintf("\n== Timing-Diagnose ==\n");
    fprintf("  Referenzmodus: %s\n", cfg.referenceTiming);
    fprintf("  Soll-Frequenz: %.2f Hz\n", cfg.rateHz);
    fprintf("  Effektive mittlere Frequenz: %.2f Hz\n", 1/mean(dt_real));
    fprintf("  Median dt: %.4f s\n", median(dt_real));
    fprintf("  Max dt: %.4f s\n", max(dt_real));
    fprintf("  End t_wall: %.3f s\n", log.t_wall(end));
    fprintf("  End t_ref : %.3f s\n", log.t_ref(end));
    fprintf("  Max RefreshFeedback-Zeit: %.4f s\n", max(log.t_feedback));
    fprintf("  Max getAction-Zeit       : %.4f s\n", max(log.t_agent));

    if max(dt_real) > 1.5 * cfg.Ts
        warning("Loop verpasst 40 Hz. Siehe Timing-Diagnose-Plot: dt_loop, RefreshFeedback, getAction.");
    end
end


%% ============ REFERENZ-FUNKTION ============

function [pos, vel] = referenceTrajectory(t, cfg)
    % Gibt Referenzposition und -geschwindigkeit im URDF/Training-Frame zurueck.
    % t darf Skalar oder Vektor sein. Rueckgabe ist N x 3.
    t = min(max(t(:), 0), cfg.maxDuration);

    x  = cfg.center(1) + cfg.r * sin(cfg.omega * t);
    y  = cfg.center(2) + cfg.yConst * t;
    z  = cfg.center(3) + cfg.r * cos(cfg.omega * t);

    vx = cfg.r * cfg.omega * cos(cfg.omega * t);
    vy = cfg.yConst * ones(size(t));
    vz = -cfg.r * cfg.omega * sin(cfg.omega * t);

    pos = [x, y, z];
    vel = [vx, vy, vz];
end

%% ============ MEX-ADAPTER-FUNKTIONEN ============

function apiHandle = kinovaOpen(cfg)
    [errCode, apiHandle, ~] = kortexApiMexInterface( ...
        'CreateRobotApisWrapper', ...
        cfg.kinovaIP, cfg.kinovaUser, cfg.kinovaPassword, ...
        cfg.sessionTimeoutMs, cfg.controlTimeoutMs);
    if errCode ~= 0
        error('CreateRobotApisWrapper fehlgeschlagen (errorCode=%d).', errCode);
    end
    fprintf('Verbunden mit Kinova Gen3 @ %s (apiHandle=%d)\n', ...
            cfg.kinovaIP, apiHandle);
end

function [q_rad, dq_rad, faultFlag, toolPoseWorld] = kinovaReadFeedback(apiHandle, cfg)
    [errCode, baseFb, actuatorFb, ~] = ...
        kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errCode ~= 0
        error('RefreshFeedback fehlgeschlagen (errorCode=%d).', errCode);
    end

    q_deg  = extractActuatorField(actuatorFb, 'position',  cfg.nJ);
    dq_deg = extractActuatorField(actuatorFb, 'velocity',  cfg.nJ);

    q_rad  = wrapPi(deg2rad(q_deg(:)));
    dq_rad = deg2rad(dq_deg(:));

    toolPoseWorld = extractToolPose(baseFb);

    faultFlag = checkFaults(actuatorFb, baseFb);
end

function tp = extractToolPose(baseFb)
    tp = [NaN NaN NaN];
    if isempty(baseFb), return; end
    if isfield(baseFb, 'tool_pose')
        s = baseFb.tool_pose;
        if isstruct(s)
            if isfield(s,'x') && isfield(s,'y') && isfield(s,'z')
                tp = [double(s.x), double(s.y), double(s.z)];
                return;
            end
        end
    end
    if isfield(baseFb, 'tool_pose_x') && isfield(baseFb, 'tool_pose_y') ...
            && isfield(baseFb, 'tool_pose_z')
        tp = [double(baseFb.tool_pose_x), ...
              double(baseFb.tool_pose_y), ...
              double(baseFb.tool_pose_z)];
    end
end

function errCode = kinovaSendVelocity(apiHandle, dq_rad, cfg)
    dq_deg = rad2deg(dq_rad(:)).';
    errCode = kortexApiMexInterface('SendJointSpeedCommand', ...
        apiHandle, cfg.speedCmdDuration, dq_deg, uint32(cfg.nJ));
end

function kinovaMoveToJoints(apiHandle, q_rad, cfg)
    q_deg = rad2deg(q_rad(:)).';
    homingSpeed_deg = rad2deg(cfg.homingSpeed);

    errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
        int32(2), homingSpeed_deg, 0, q_deg);

    if errCode ~= 0
        warning(['Homing mit Speed-Constraint fehlgeschlagen ' ...
                 '(errorCode=%d). Versuche Default-Modus.'], errCode);
        errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
            int32(0), 0, 0, q_deg);
        if errCode ~= 0
            error('ReachJointAngles fehlgeschlagen (errorCode=%d).', errCode);
        end
    end
end

function waitForJointTarget(apiHandle, q_target_rad, cfg)
    fprintf('   Warte auf Homing-Ende...');
    tStart = tic;
    stable = 0;
    while toc(tStart) < cfg.homingWaitTimeout
        [q, ~, faultFlag] = kinovaReadFeedback(apiHandle, cfg);
        if faultFlag
            error('Fault waehrend Homing -- Abbruch.');
        end
        err = wrapPi(q - q_target_rad);
        if all(abs(err) < cfg.homingPosTol)
            stable = stable + 1;
            if stable >= 5
                fprintf(' OK (%.1fs)\n', toc(tStart));
                return;
            end
        else
            stable = 0;
        end
        pause(0.05);
    end
    warning('Homing-Timeout nach %.1fs.', cfg.homingWaitTimeout);
end

function safeShutdown(apiHandle, nJ)
    try
        fprintf('\n[safeShutdown] Sende Zero-Velocity...\n');
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, 0, zeros(1, nJ), uint32(nJ));
        pause(0.05);
    catch
    end
    try
        kortexApiMexInterface('StopAction', apiHandle);
    catch
    end
    try
        kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
        fprintf('[safeShutdown] MEX-Session geschlossen.\n');
    catch
    end
end

%% ============ HILFSFUNKTIONEN ============

function vals = extractActuatorField(actuatorFb, fieldName, nJ)
    vals = zeros(1, nJ);
    if numel(actuatorFb) == 1 && isfield(actuatorFb, fieldName) ...
            && numel(actuatorFb.(fieldName)) >= nJ
        vals = double(actuatorFb.(fieldName)(1:nJ));
        return;
    end
    for i = 1:min(nJ, numel(actuatorFb))
        if isfield(actuatorFb, fieldName)
            vals(i) = double(actuatorFb(i).(fieldName));
        end
    end
end

function flag = checkFaults(actuatorFb, baseFb)
    flag = false;

    if ~isempty(actuatorFb) && isfield(actuatorFb, 'fault_bank_a')
        for i = 1:numel(actuatorFb)
            if any(double(actuatorFb(i).fault_bank_a) ~= 0)
                flag = true; return;
            end
        end
    end
    if ~isempty(actuatorFb) && isfield(actuatorFb, 'fault_bank_b')
        for i = 1:numel(actuatorFb)
            if any(double(actuatorFb(i).fault_bank_b) ~= 0)
                flag = true; return;
            end
        end
    end

    if ~isempty(baseFb)
        if isfield(baseFb, 'fault_bank_a') && ...
                any(double(baseFb.fault_bank_a) ~= 0)
            flag = true; return;
        end
        if isfield(baseFb, 'fault_bank_b') && ...
                any(double(baseFb.fault_bank_b) ~= 0)
            flag = true; return;
        end
    end
end

function wrapped = wrapPi(angle)
    wrapped = mod(angle + pi, 2*pi) - pi;
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

function [dq_out, state] = applyTrainingPipeline(dq_raw, state, cfg)
    dq_s = max(min(dq_raw, cfg.satUpper), cfg.satLower);
    state.yFilt = cfg.filt_num * dq_s + cfg.filt_den * state.yFilt;
    dq_f = state.yFilt;
    dq_delta = dq_f - state.dqPrev;
    maxDelta = cfg.slewRate * cfg.Ts;
    dq_delta = max(min(dq_delta, maxDelta), -maxDelta);
    dq_out   = state.dqPrev + dq_delta;
    state.dqPrev = dq_out;
end

function log = initLog(n, nJ)
    log = struct();
    % log.t bleibt aus Kompatibilitaetsgruenden die verwendete Referenzzeit.
    log.t              = zeros(n, 1);
    log.t_ref          = zeros(n, 1);
    log.t_wall         = zeros(n, 1);
    log.dt_loop        = zeros(n, 1);
    log.dt_since_send  = nan(n, 1);
    log.t_feedback     = zeros(n, 1);
    log.t_agent        = zeros(n, 1);
    log.t_send         = zeros(n, 1);
    log.timing_overrun = false(n, 1);
    log.q              = zeros(n, nJ);
    log.dq             = zeros(n, nJ);
    log.dq_raw         = zeros(n, nJ);
    log.dq_cmd         = zeros(n, nJ);
    log.dq_filt        = zeros(n, nJ);
    log.yFilt          = zeros(n, nJ);
    log.ee_pos         = zeros(n, 3);
    log.ee_tool        = zeros(n, 3);
    log.ee_kortex      = nan(n, 3);
    log.ee_ref         = zeros(n, 3);
    log.ep             = zeros(n, 3);
    log.ep_norm        = zeros(n, 1);
end

function log = truncateLog(log, n)
    if n < 1, return; end
    log.t              = log.t(1:n);
    log.t_ref          = log.t_ref(1:n);
    log.t_wall         = log.t_wall(1:n);
    log.dt_loop        = log.dt_loop(1:n);
    log.dt_since_send  = log.dt_since_send(1:n);
    log.t_feedback     = log.t_feedback(1:n);
    log.t_agent        = log.t_agent(1:n);
    log.t_send         = log.t_send(1:n);
    log.timing_overrun = log.timing_overrun(1:n);
    log.q              = log.q(1:n, :);
    log.dq             = log.dq(1:n, :);
    log.dq_raw         = log.dq_raw(1:n, :);
    log.dq_cmd         = log.dq_cmd(1:n, :);
    log.dq_filt        = log.dq_filt(1:n, :);
    log.yFilt          = log.yFilt(1:n, :);
    log.ee_pos         = log.ee_pos(1:n, :);
    log.ee_tool        = log.ee_tool(1:n, :);
    log.ee_kortex      = log.ee_kortex(1:n, :);
    log.ee_ref         = log.ee_ref(1:n, :);
    log.ep             = log.ep(1:n, :);
    log.ep_norm        = log.ep_norm(1:n);
end

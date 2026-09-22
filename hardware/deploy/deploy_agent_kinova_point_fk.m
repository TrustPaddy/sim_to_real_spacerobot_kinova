% %% deploy_agent_kinova_p2p_10hz_fk.m  (V3.1 - Point-to-Point, 10 Hz, MIT URDF-FK)
% Deployt einen trainierten PPO-Agenten (Point-to-Point Task) auf den echten
% Kinova Gen3 via kortexApiMexInterface (MEX). Repliziert die Simulink-
% Post-Action-Pipeline (Saturation -> Discrete Filter -> Rate Limiter) 1:1
% aus dem Trainings-Modell und wendet darauf die Hardware-Schutzschicht an.
%
% AENDERUNG gegenueber V3.0 (deploy_agent_kinova_p2p_10hz.m, "ohne URDF"):
%   - EE-Zustand kommt jetzt wieder aus der URDF-FORWARD-KINEMATIK auf den
%     gemessenen Gelenkwinkeln q (wie im Trajektorien-Deploy-Skript), NICHT
%     mehr aus baseFb.tool_pose / baseFb.tool_twist:
%       ee_pos = getTransform(robot, q, eeBodyName)            -> Translation
%       ee_vel = geometricJacobian(robot, q, eeBodyName)(4:6,:) * dq  (linear)
%     -> Damit lebt der EE-Zustand direkt im URDF-(Trainings-)Frame; der
%        realBaseOffset (real Base -> URDF Base) entfaellt komplett, weil
%        kein realer Base-Frame mehr beteiligt ist.
%   - WICHTIG: robot.DataFormat = 'row'  (== Training!). q/dq werden als
%     Zeilenvektoren an getTransform/geometricJacobian uebergeben.
%   - tool_pose/tool_twist werden gar nicht mehr gelesen; die zugehoerige
%     Roh-Struktur-Diagnose und die Extractor-Helfer entfallen.
%   - Der Dry-Run ist dadurch ein ECHTER kinematischer Check: q wird aus den
%     dq-Kommandos integriert und die EE-Pose per FK daraus berechnet
%     (vorher blieb die EE-Pose im Dry-Run statisch).
%   - e_ori bleibt fest [0;0;0] (Basis-Orientierung, fester Kinova).
%
% AUS V3.0 / V2.2 UEBERNOMMEN:
%   - Point-to-Point Task (ref_pos = const, ref_vel = 0), 10 Hz (= Ts_agent)
%   - Trainings-Pipeline-Replikation (Sat -> IIR-Filter -> RateLim)
%   - HW-Hard-Cap, Soft-Limit-Bremsung, OOD-Stopp, Fault-Check
%   - guarded_wall-Zeitbasis, Timing-Watchdog, Diagnose-Logs
%   - Konvergenz-Erkennung mit Frueh-Stopp, optionales Homing
%   - onCleanup mit Zero-Velocity + DestroyRobotApisWrapper

clear; clc; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();

% ---- SICHERHEIT: Dry-Run + Pre-Flight ----
cfg.dryRun            = false;
cfg.preFlightRequired = false;

% ---- Lauf-Identifikation (fuer run_logger / C3) ----
cfg.seed = 1;   % manuell pro Lauf hochzaehlen

% ---- Agent laden ----
% TODO: Pfad an deinen neuen P2P-Agenten anpassen.
cfg.agentFile      = "";
cfg.agentDir       = "../SavedAgents/MotionProfile/Point/test_agent_rand2.mat";
cfg.expectedObsDim = 29;         % unveraendert: 3+3+7+7+3+3+3

% ---- URDF / Forward Kinematics (ERSETZT tool_pose/tool_twist) ----
% EE-Pose, -Rotation und -Geschwindigkeit kommen aus der URDF-FK auf den
% gemessenen Gelenkwinkeln q -- exakt wie im Trajektorien-Deploy-Skript.
% >>> WICHTIG <<<
%   * cfg.urdfFile MUSS dieselbe URDF sein wie im Training / Traj.-Deploy.
%   * cfg.eeBodyName MUSS der EE-Body-Name in genau dieser URDF sein.
%   * cfg.dataFormat MUSS 'row' sein (== Training, NICHT 'column').
% Am einfachsten: die importrobot-/Body-Namen-Zeilen 1:1 aus dem
% funktionierenden Trajektorien-Deploy-Skript uebernehmen.
cfg.urdfFile    = "../SpaceKinova.urdf";              % TODO: Pfad zur Trainings-URDF eintragen
cfg.eeBodyName  = "kinova_end_effector_link";   % TODO: EE-Body-Name pruefen (z.B. 'EndEffector', 'tool_frame', 'Bracelet_Link')
cfg.dataFormat  = 'row';           % == Training (NICHT 'column')

% ---- EE-/Basis-Orientierungsfehler e_ori ----
% e_ori ist eine BASIS-Groesse (Orientierungsfehler des frei schwebenden
% SpaceKinova-Bodies). Auf dem fest montierten realen Kinova ist die Basis-
% Orientierung konstant -> e_ori ist IMMER [0;0;0]. Wird im Loop fest
% verdrahtet (KEINE FK noetig, kein Schalter).

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
cfg.speedScale = 0.5;

% ---- Trainings-Pipeline-Parameter (IDENTISCH zu Training/V2.2) ----
cfg.satUpper = [0.9774; 0.9774; 0.9774; 0.9774; 0.5774; 0.5774; 0.5774];
cfg.satLower = -cfg.satUpper;
cfg.filt_num = 0.05;
cfg.filt_den = 0.95;
cfg.slewRate = 0.5;

% ---- HW-Schutz ----
cfg.qSoftMargin = deg2rad(10);

% ---- Steuerung: 10 Hz (= Trainings-Ts_agent) ----
cfg.rateHz          = 10;
cfg.Ts              = 1/cfg.rateHz;       % = 0.1 s
cfg.maxDuration     = 60.0;
cfg.watchdogTimeout = 0.25;

% ---- Referenz-Zeitbasis ----
cfg.referenceTiming      = "guarded_wall";
cfg.maxRefAdvanceFactor  = 1.5;
cfg.stopOnTimingOverrun  = true;
cfg.rateControlInDryRun  = true;

% ---- Point-to-Point Ziel (Trainings-/URDF-Frame) ----
cfg.target_pos = [0.479; -0.005; 1.036];

% ---- Fehler-Konvention (fest, = Training) ----
% ep = ee_ist - ee_ziel, ev = vee - vref. Im Loop fest verdrahtet.

% ---- Start-Konfiguration (q_start_anchor aus Training) ----
cfg.q_start_anchor = deg2rad([0; 15; 180; -130; 0; 55; 90]);

% ---- Optionales automatisches Homing ----
cfg.doHoming          = true;
cfg.homingSpeed       = deg2rad(10);     % rad/s
cfg.homingWaitTimeout = 20;
cfg.homingPosTol      = deg2rad(1.0);

% ---- Konvergenz-Erkennung (Frueh-Stopp) ----
cfg.stopOnConvergence    = true;
cfg.convergenceDistM     = 0.02;   % d_success aus Reward (EE-Ziel-Abstand)
cfg.convergenceVelMPerS  = 0.03;   % v_success aus Reward (||ev||, ref_v=0)
cfg.convergenceOriRad    = 0.05;   % ori_success aus Reward (EE-Orient.fehler)
cfg.convergenceWbaseRadS = 0.05;   % w_success aus Reward (immer ~0 bei fixed base)
cfg.convergenceDwellS    = 0.5;    % Dwell-Filter gegen Einzel-Step-Glueck

% ---- Observation Limits (== Training!) ----
cfg.ePLim   = 1.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

% ---- OOD-/Failure-Schwellen (analog zu rewardFcn Failure-Termination) ----
cfg.oodThreshold   = 2.0;      % m: d_failure aus Reward
cfg.oodOriRad      = 1.0;      % rad: ori_failure aus Reward (~57 deg)
cfg.divergenceWarn = 0.8;      % m: erste Warnung deutlich vor OOD

%% =========================
%  1) PRE-FLIGHT-GATE
%  =========================
if ~cfg.dryRun && cfg.preFlightRequired
    error(['Pre-Flight fehlt. Bitte erst in Simulink mit identischer ' ...
           'Observation-Struktur + 10 Hz laufen lassen und Return/KPIs ' ...
           'pruefen. Danach cfg.preFlightRequired = false setzen.']);
end

%% =========================
%  2) AGENT LADEN + OBS-KONSISTENZ
%  =========================
if strlength(cfg.agentFile) == 0
    pattern = fullfile(cfg.agentDir);
    d = dir(pattern);
    if isempty(d)
        error("Kein Agent gefunden unter: %s", pattern);
    end
    [~, idx] = max([d.datenum]);
    cfg.agentFile = fullfile(d(idx).folder, d(idx).name);
end

fprintf("Lade Agent: %s\n", cfg.agentFile);
loaded = load(cfg.agentFile);
assert(isfield(loaded, 'agent'), ...
    "Datei enthaelt kein Feld 'agent': %s", cfg.agentFile);
agent = loaded.agent;

agent.UseExplorationPolicy = false;

obsInfo      = getObservationInfo(agent);
actualObsDim = obsInfo.Dimension(1);
assert(actualObsDim == cfg.expectedObsDim, ...
    "ObservationDim Mismatch: Agent=%d, Deploy=%d.", ...
    actualObsDim, cfg.expectedObsDim);

%% =========================
%  3) OBSERVATION LIMITS
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
%  3b) URDF / FK LADEN + FK-SANITY
%  =========================
assert(strlength(cfg.urdfFile) > 0, ...
    ["cfg.urdfFile ist leer. Bitte Pfad zur Trainings-URDF setzen " ...
     "(identisch zum Trajektorien-Deploy-Skript)."]);
fprintf("Lade URDF: %s\n", cfg.urdfFile);
robot = importrobot(cfg.urdfFile);
robot.DataFormat = cfg.dataFormat;   % MUSS 'row' sein (== Training)
fprintf("   URDF geladen. DataFormat='%s', EE-Body='%s'\n", ...
        robot.DataFormat, cfg.eeBodyName);

% FK-Test auf dem Anchor -- faengt falschen Body-Namen / DataFormat frueh ab.
try
    T_test = getTransform(robot, cfg.q_start_anchor(:).', cfg.eeBodyName);
    J_test = geometricJacobian(robot, cfg.q_start_anchor(:).', cfg.eeBodyName);
    fprintf("   FK-Test OK: EE @ q_start_anchor = [%s] m\n", ...
            join(string(round(T_test(1:3,4).',3)), ", "));
    assert(isequal(size(J_test), [6, cfg.nJ]), ...
        "geometricJacobian liefert %s, erwartet [6 %d].", ...
        mat2str(size(J_test)), cfg.nJ);
catch ME
    error(['FK-Test fehlgeschlagen. Pruefe cfg.eeBodyName, cfg.urdfFile ' ...
           'und cfg.dataFormat. Fehler: %s'], ME.message);
end

%% =========================
%  4) MEX-VERBINDUNG + DIAGNOSE
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
    if ~isempty(baseFb)
        fprintf("   baseFb fields = {%s}\n", strjoin(fieldnames(baseFb), ', '));
    end

    % EE-Zustand kommt jetzt aus FK -> tool_pose/tool_twist werden NICHT
    % mehr benoetigt und auch nicht mehr geprueft.

    % Erste Messung: nur Joint-Zustand lesen, EE-Zustand per FK berechnen.
    [q0, dq0, ~] = kinovaReadFullFeedback(apiHandle, cfg);
    [eePos0, eeVel0] = computeEEFromFK(robot, q0, dq0, cfg);
    fprintf("Verbindung OK.\n");
    fprintf("   q0 [rad]      = [%s]\n", join(string(round(q0,3)), ", "));
    fprintf("   ee_pos (FK)   = [%s] m   (URDF-Frame)\n", ...
            join(string(round(eePos0,3)), ", "));
    fprintf("   ee_vel (FK)   = [%s] m/s\n", ...
            join(string(round(eeVel0,3)), ", "));
    if any(~isfinite(eePos0)) || any(~isfinite(eeVel0))
        error('FK lieferte NaN/Inf. Pruefe URDF / eeBodyName.');
    end
else
    fprintf("\n=== DRY-RUN MODUS ===\n");
    fprintf("Keine MEX-Verbindung. Kommandos werden nur angezeigt.\n");
    fprintf("EE-Pose wird per FK aus integriertem q berechnet.\n\n");
    q0  = cfg.q_start_anchor;
    dq0 = zeros(cfg.nJ, 1);
end

%% =========================
%  5) OPTIONALES HOMING
%  =========================
if cfg.doHoming && ~cfg.dryRun
    fprintf("\n== Homing zu q_start_anchor ==\n");
    fprintf("   Ziel: [%s] deg\n", ...
            join(string(round(rad2deg(cfg.q_start_anchor),1)), ", "));
    fprintf("   Speed: %.1f deg/s\n", rad2deg(cfg.homingSpeed));
    kinovaMoveToJoints(apiHandle, cfg.q_start_anchor, cfg);
    waitForJointTarget(apiHandle, cfg.q_start_anchor, cfg);
    pause(0.5);
    fprintf("   Homing abgeschlossen.\n");
elseif cfg.doHoming && cfg.dryRun
    fprintf("\n[Dry-Run] Homing-Schritt wird uebersprungen.\n");
else
    fprintf("\n[INFO] Auto-Homing deaktiviert -- Roboter muss manuell positioniert sein.\n");
end

%% =========================
%  6) STARTPOSITION DIAGNOSE
%  =========================
if ~cfg.dryRun
    [q_now, dq_now, ~] = kinovaReadFullFeedback(apiHandle, cfg);
else
    q_now  = cfg.q_start_anchor;
    dq_now = zeros(cfg.nJ, 1);
end

[ee_pos_start, ~] = computeEEFromFK(robot, q_now, dq_now, cfg);
err_start = norm(ee_pos_start - cfg.target_pos);
fprintf("\nStartabstand zum Ziel: %.4f m (URDF-Frame, FK)\n", err_start);
fprintf("  EE start (FK):  [%s] m\n", ...
        join(string(round(ee_pos_start,3)), ", "));
fprintf("  Target  (URDF): [%s] m\n", ...
        join(string(round(cfg.target_pos,3)), ", "));

if err_start > cfg.ePLim
    warning(['Startabstand %.3f m > ePLim=%.3f m. Die anfaengliche ep ' ...
             'wird im Obs-Clipping abgeschnitten -> Agent sieht einen ' ...
             'verzerrten Fehler. Erwaege groesseres ePLim oder Roboter ' ...
             'naeher an Ziel positionieren.'], err_start, cfg.ePLim);
end

%% =========================
%  7) SICHERHEITSHINWEIS + CAP-ANALYSE
%  =========================
fprintf("\n== SICHERHEITSHINWEIS ==\n");
fprintf(" - Arbeitsraum frei, E-Stop bereit\n");
fprintf(" - Task: Point-to-Point, Target = [%s] m (URDF-Frame)\n", ...
        join(string(round(cfg.target_pos,3)), ", "));
fprintf(" - Agent-Pipeline: Trainings-identisch (sat=[%s])\n", ...
        join(string(cfg.satUpper), " "));
fprintf(" - HW-Hard-Cap: safetyFactor=%.2f -> dqHwCap=[%s] rad/s\n", ...
        cfg.safetyFactor, join(string(round(cfg.dqHwCap, 3)), ", "));
fprintf(" - Speed-Scale (uniform, nach HW-Cap): %.2f\n", cfg.speedScale);
fprintf(" - Deterministische Policy (UseExplorationPolicy=false)\n");
fprintf(" - Sample-Rate: %.0f Hz (Ts=%.3fs, == Trainings-Ts_agent)\n", ...
        cfg.rateHz, cfg.Ts);
fprintf(" - Max-Laufzeit: %.1f s\n", cfg.maxDuration);
fprintf(" - Stopp-Kriterien:\n");
fprintf("     * Konvergenz (alle 4 gleichzeitig, %.2fs Dwell):\n", ...
        cfg.convergenceDwellS);
fprintf("         ||ep||    < %.4f m\n",   cfg.convergenceDistM);
fprintf("         ||ev||    < %.4f m/s\n", cfg.convergenceVelMPerS);
fprintf("         ||e_ori|| < %.4f rad\n", cfg.convergenceOriRad);
fprintf("         ||w_base||< %.4f rad/s\n", cfg.convergenceWbaseRadS);
fprintf("     * OOD/Failure:\n");
fprintf("         ||ep||    > %.3f m\n",   cfg.oodThreshold);
fprintf("         ||e_ori|| > %.3f rad\n", cfg.oodOriRad);
fprintf("     * Timing    : dt > %.3f s (watchdog)\n", cfg.watchdogTimeout);
fprintf(" - EE-Zustand kommt aus URDF-FK (getTransform/geometricJacobian).\n");
fprintf(" - URDF: %s  | EE-Body: '%s' | DataFormat: '%s'\n", ...
        cfg.urdfFile, cfg.eeBodyName, robot.DataFormat);
fprintf(" - e_ori = [0 0 0] (Basis-Orientierung, fester Kinova)\n");
fprintf(" - Lauf-Seed: %d\n", cfg.seed);
fprintf(" - Dry-Run: %s\n\n", string(cfg.dryRun));

fprintf("== Cap-Analyse (Training-Sat vs. HW-Cap) ==\n");
fprintf(" Joint | satUpper | dqHwCap | Cap beisst? | max dq_cmd nach scale\n");
fprintf(" ------|----------|---------|-------------|-----------------------\n");
anyBites = false;
for j = 1:cfg.nJ
    sat_j  = cfg.satUpper(j);
    cap_j  = cfg.dqHwCap(j);
    eff_j  = min(sat_j, cap_j);
    bites  = (sat_j > 0) && (cap_j < sat_j - 1e-6);
    if bites, anyBites = true; end
    max_cmd = cfg.speedScale * eff_j;
    fprintf("  J%d   |  %5.3f  |  %5.3f  |     %s     |  %5.3f rad/s (%5.2f deg/s)\n", ...
        j, sat_j, cap_j, ...
        ternary(bites, "JA", "nein"), max_cmd, rad2deg(max_cmd));
end
if anyBites
    warning(["HW-Cap beisst auf aktiven Joints -> Training-Ratios werden " ...
             "verzerrt. Erhoehe cfg.safetyFactor oder reduziere stattdessen " ...
             "cfg.speedScale."]);
else
    fprintf("  OK: HW-Cap inaktiv auf allen aktiven Joints -> Ratios bleiben erhalten.\n");
end
fprintf("\n");

if ~cfg.dryRun
    input("ENTER zum Starten (CTRL+C zum Abbrechen)... ", "s");
end

%% =========================
%  8) WARM-UP VOR DEM ZEITSTART
%  =========================
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
        [q_now, dq0, faultFlagWarm] = kinovaReadFullFeedback(apiHandle, cfg);
        if faultFlagWarm
            error('Fault waehrend Warm-up -- Abbruch.');
        end
        pause(0.02);
    end
    fprintf("  Feedback warm-up OK\n");
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
nStepsPlan = ceil(cfg.maxDuration * cfg.rateHz) + 1;
nStepsMax  = nStepsPlan + ceil(2.0 * cfg.rateHz);
log = initLog(nStepsMax, cfg.nJ);

q  = q_now;
dq = dq0;
warned_divergence = false;
convergedSinceWall = NaN;   % toc-Zeit, ab der ||ep|| innerhalb tol liegt

fprintf("== Starte Deployment-Loop: mode=%s, %.0f Hz, max %.2f s ==\n", ...
        cfg.referenceTiming, cfg.rateHz, cfg.maxDuration);
fprintf("   Watchdog: dt > %.3f s -> %s\n", cfg.watchdogTimeout, ...
        ternary(cfg.stopOnTimingOverrun, "STOPP", "nur Warnung"));
fprintf("   guarded_wall: max dt_ref/step = %.4f s\n\n", ...
        cfg.maxRefAdvanceFactor * cfg.Ts);

r = rateControl(cfg.rateHz);
try, reset(r); catch, end

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

    % --- Watchdog seit letztem Send ---
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

    % --- Referenz-Zeit (fuer Logging; ref_pos selbst ist konstant) ---
    switch string(cfg.referenceTiming)
        case "wall"
            tRef = min(tWallLoop, cfg.maxDuration);
        case "sample"
            tRef = min((k - 1) * cfg.Ts, cfg.maxDuration);
        case "guarded_wall"
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

    if tRef >= cfg.maxDuration && k > 1
        fprintf("\n[INFO] Max-Dauer erreicht: t_ref=%.3f s\n", tRef);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        break;
    end

    % --- Joint-Zustand lesen (Live) bzw. integrieren (Dry-Run) ---
    tBeforeFeedback = toc(tRunStart);
    if ~cfg.dryRun
        [q, dq, faultFlag] = kinovaReadFullFeedback(apiHandle, cfg);
        if faultFlag
            fprintf('\n[FAULT] Kinova-Fault erkannt -- Stopp\n');
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
            % k_end behaelt den letzten gueltigen Wert (k-1).
            break;
        end
    else
        % Dry-Run: q aus dem zuletzt gesendeten dq_cmd fortschreiben. Die
        % EE-Pose folgt dann unten konsistent per FK (echter kinematischer
        % Check, KEINE Dynamik-Simulation -- Plant-Traegheit fehlt).
        if exist('dq_cmd', 'var')
            dq = dq_cmd;                 % letztes Kommando als "gemessene" dq
            q  = q + dq * cfg.Ts;        % kein wrap -> Anchor-relativ konsistent
        end
    end
    tAfterFeedback = toc(tRunStart);

    % --- EE-Zustand aus URDF-FK (Live UND Dry-Run identisch) ---
    [ee_pos, ee_vel] = computeEEFromFK(robot, q, dq, cfg);

    % --- Konstante Referenz (Point-to-Point) ---
    ref_pos = cfg.target_pos;
    ref_vel = zeros(3, 1);

    % --- Positions-/Geschwindigkeitsfehler ---
    % Trainings-Konvention (bestaetigt): ee_diff = ee_ist - ee_ziel.
    ep = ee_pos(:) - ref_pos;
    ev = ee_vel(:) - ref_vel;

    % --- Basis-Groessen v_base, w_base, e_ori: fester Kinova -> alle 0 ---
    % v_base/w_base = lin./ang. Geschw. des SpaceKinova-Bodies, e_ori =
    % dessen Orientierungsfehler. Auf fester Basis konstant -> [0;0;0].
    v_base = zeros(3, 1);
    w_base = zeros(3, 1);
    e_ori  = zeros(3, 1);

    % --- Observation (29D) ---
    obs = [ep; ev; q; dq; v_base; w_base; e_ori];

    % --- Eskalations-Warnung ---
    if norm(ep) > cfg.divergenceWarn && ~warned_divergence
        fprintf('\n[WARN k=%d] ||ep||=%.3f m > %.2f m -- Divergenz erkannt.\n', ...
                k, norm(ep), cfg.divergenceWarn);
        warned_divergence = true;
    end

    % --- OOD-/Failure-Check (analog zu rewardFcn Failure-Termination) ---
    if norm(ep) > cfg.oodThreshold
        fprintf(['\n[STOPP k=%d] Positionsfehler %.3f m > Schwelle %.3f m ' ...
                 '(d_failure / Out-of-Distribution)\n'], ...
                 k, norm(ep), cfg.oodThreshold);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        break;   % k_end behaelt letzten gueltigen Wert (Zeile k ungeloggt)
    end
    if norm(e_ori) > cfg.oodOriRad
        fprintf(['\n[STOPP k=%d] Orientierungsfehler %.3f rad > Schwelle ' ...
                 '%.3f rad (ori_failure)\n'], ...
                 k, norm(e_ori), cfg.oodOriRad);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        break;   % k_end behaelt letzten gueltigen Wert (Zeile k ungeloggt)
    end

    % --- Konvergenz-Erkennung (Frueh-Stopp, 1:1 wie rewardFcn Success) ---
    if cfg.stopOnConvergence
        dist   = norm(ep);            % EE-Ziel-Abstand
        v_ee   = norm(ev);            % EE-Geschw. (ev = -ee_vel da ref_v=0)
        ori    = norm(e_ori);         % EE-Orientierungsfehler
        w_norm = norm(w_base);        % Basis-Winkelgeschw. (=0 bei fixed base)

        converged = (dist   < cfg.convergenceDistM)     && ...
                    (v_ee   < cfg.convergenceVelMPerS)  && ...
                    (ori    < cfg.convergenceOriRad)    && ...
                    (w_norm < cfg.convergenceWbaseRadS);

        if converged
            if isnan(convergedSinceWall)
                convergedSinceWall = tWallLoop;
            end
            dwell = tWallLoop - convergedSinceWall;
            if dwell >= cfg.convergenceDwellS
                fprintf(['\n[SUCCESS k=%d, t=%.2fs] Alle 4 Reward-Kriterien ' ...
                         'erfuellt fuer %.2fs:\n' ...
                         '   ||ep||=%.4f m  (< %.4f)\n' ...
                         '   ||ev||=%.4f m/s (< %.4f)\n' ...
                         '   ||e_ori||=%.4f rad (< %.4f)\n' ...
                         '   ||w_base||=%.4f rad/s (< %.4f)\n' ...
                         '   -> Frueh-Stopp, Bewegung auf 0.\n'], ...
                         k, tWallLoop, dwell, ...
                         dist,   cfg.convergenceDistM, ...
                         v_ee,   cfg.convergenceVelMPerS, ...
                         ori,    cfg.convergenceOriRad, ...
                         w_norm, cfg.convergenceWbaseRadS);
                if ~cfg.dryRun
                    kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
                end
                log = logRow(log, k, tRef, tWallLoop, dtLoop, dtSinceSend, ...
                    tAfterFeedback - tBeforeFeedback, NaN, NaN, ...
                    timingOverrun, q, dq, zeros(cfg.nJ,1), zeros(cfg.nJ,1), ...
                    state.yFilt, ee_pos, ref_pos, ep, ee_pos);
                k_end = k;
                break;
            end
        else
            convergedSinceWall = NaN;   % Reset bei Verlassen der Success-Region
        end
    end

    % --- Obs-Clipping ---
    obs = max(min(obs, obsHigh), obsLow);

    % --- Agent abfragen ---
    tBeforeAgent = toc(tRunStart);
    action = getAction(agent, {obs});
    tAfterAgent = toc(tRunStart);
    dq_raw = action{1}(:);
    dq_raw = max(min(dq_raw, 1), -1);  % defensives Clipping

    % --- Trainings-Pipeline (Sat -> Filter -> RateLim) ---
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

    % --- Multi-Step-Dump (Debug) ---
    if any(k == [1, 2, 3, 5, 10, 20, 50, 100, 150, 200])
        fprintf('\n=== STEP %d DUMP (t_wall=%.3fs, t_ref=%.3fs, dt=%.4fs) ===\n', ...
                k, tWallLoop, tRef, dtLoop);
        fprintf('  Feedback-Zeit = %.4f s, Agent-Zeit = %.4f s\n', ...
                tAfterFeedback - tBeforeFeedback, tAfterAgent - tBeforeAgent);
        fprintf('  q       = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad\n', q);
        fprintf('  ee_pos  = [%+.4f %+.4f %+.4f] m (FK, URDF)\n', ee_pos);
        fprintf('  target  = [%+.4f %+.4f %+.4f] m (URDF)\n', cfg.target_pos);
        fprintf('  ep      = [%+.4f %+.4f %+.4f] m, ||ep||=%.4f\n', ep, norm(ep));
        fprintf('  ee_vel  = [%+.4f %+.4f %+.4f] m/s (FK Jacobi)\n', ee_vel);
        fprintf('  e_ori   = [%+.4f %+.4f %+.4f] (Basis, fix=0)\n', e_ori);
        fprintf('  dq_raw  = [%+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f]\n', dq_raw);
        fprintf('  yFilt   = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f]\n', state.yFilt);
        fprintf('  dq_cmd  = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad/s\n', dq_cmd);
        fprintf('  dq_meas = [%+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f] rad/s\n', dq);
        fprintf('=========================\n');
    end

    % --- Senden / Anzeigen ---
    if cfg.dryRun
        if mod(k, cfg.rateHz) == 1
            fprintf("t_wall=%.2fs | t_ref=%.2fs | ||ep||=%.3f | dq_cmd=[%s] rad/s\n", ...
                tWallLoop, tRef, norm(ep), ...
                join(string(round(dq_cmd, 4)), " "));
        end
    else
        sendErr = kinovaSendVelocity(apiHandle, dq_cmd, cfg);
        if sendErr ~= 0
            warning('SendJointSpeedCommand errorCode=%d -- Stopp.', sendErr);
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
            break;   % k_end behaelt letzten gueltigen Wert (Zeile k ungeloggt)
        end
    end

    tAfterSend = toc(tRunStart);
    tLastSendWall = tAfterSend;

    % --- Logging ---
    log = logRow(log, k, tRef, tWallLoop, dtLoop, dtSinceSend, ...
        tAfterFeedback - tBeforeFeedback, ...
        tAfterAgent - tBeforeAgent, ...
        tAfterSend - tAfterAgent, ...
        timingOverrun, q, dq, dq_raw, dq_cmd, state.yFilt, ...
        ee_pos, ref_pos, ep, ee_pos);

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
if k_end < 1
    warning('Kein gueltiger Log-Step vorhanden. k_end=1 fuer Diagnose.');
    k_end = 1;
end
log = truncateLog(log, k_end);

ep_norms = vecnorm(log.ep, 2, 2);
fprintf("Position Tracking RMSE: %.4f m\n", sqrt(mean(ep_norms.^2)));
fprintf("Position Tracking Max:  %.4f m\n", max(ep_norms));
fprintf("End-Position-Error:     %.4f m\n", ep_norms(end));

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
    if cfg.satUpper(j) == 0, continue; end
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

% --- Speichern: alter Diagnose-Stil ---
logFile = sprintf("deploy_log_p2p_fk_%s.mat", ...
                  string(datetime('now'), 'yyyyMMdd_HHmmss'));
save(logFile, "cfg", "log");
fprintf("Log gespeichert: %s\n", logFile);

% --- Speichern: run_logger-Format ---
if exist('run_logger', 'file') == 2
    data = struct();
    data.t            = log.t;
    data.t_ref        = log.t_ref;
    data.t_wall       = log.t_wall;
    data.dt_loop      = log.dt_loop;
    data.dt_since_send = log.dt_since_send;
    data.q_measured   = rad2deg(log.q);
    data.dq_cmd       = rad2deg(log.dq_cmd);
    data.dq_measured  = rad2deg(log.dq);
    data.dq_raw       = log.dq_raw;
    data.dq_filt      = log.dq_filt;
    data.ee_measured  = log.ee_pos;
    data.ee_kortex    = log.ee_kortex;  % == ee_pos (FK), kein separater Kortex-EE mehr
    data.ee_ref       = log.ee_ref;
    data.ep           = log.ep;
    data.ep_norm      = log.ep_norm;

    meta = struct();
    meta.label         = sprintf('agent_p2p_fk_seed%d', cfg.seed);
    meta.source        = 'real';
    meta.startpose     = rad2deg(q_now)';
    meta.startposeName = 'q_start_anchor';
    meta.seed          = cfg.seed;
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
    meta.endEpM        = ep_norms(end);
    meta.target_pos    = cfg.target_pos.';
    meta.usedURDF      = true;
    meta.urdfFile      = cfg.urdfFile;
    meta.eeBodyName    = cfg.eeBodyName;
    meta.comment       = 'P2P Agent-Deploy 10 Hz, MIT URDF-FK (getTransform/geometricJacobian)';

    run_logger(data, meta);
else
    fprintf("\n[Hinweis] run_logger.m nicht im Pfad - C3-Log uebersprungen.\n");
end

%% =========================
%  12) PLOTS
%  =========================
figure('Name', 'P2P Deployment: EE Position vs. Ziel');
subplot(2,1,1);
plot(log.t_wall, log.ee_pos, '-', 'LineWidth', 1.3); hold on;
yline(cfg.target_pos(1), '--', 'target_x');
yline(cfg.target_pos(2), '--', 'target_y');
yline(cfg.target_pos(3), '--', 'target_z');
xlabel('t_{wall} [s]'); ylabel('EE-Position [m]');
legend('x','y','z','Location','best');
title('EE-Position (aus URDF-FK) vs. konstantes Ziel'); grid on;
subplot(2,1,2);
plot(log.t_wall, ep_norms, 'b-', 'LineWidth', 1.3); hold on;
yline(cfg.convergenceDistM, 'g--', 'conv dist');
yline(cfg.divergenceWarn,   'm--', 'div warn');
yline(cfg.oodThreshold,     'r--', 'OOD/failure');
xlabel('t_{wall} [s]'); ylabel('||ep|| [m]');
title('Positions-Fehler-Norm'); grid on;

figure('Name', 'P2P Deployment: Timing-Diagnose');
subplot(3,1,1);
plot(log.t_wall, log.t_ref, 'LineWidth', 1.2); hold on;
plot(log.t_wall, log.t_wall, ':');
xlabel('t_{wall} [s]'); ylabel('t_{ref} [s]');
legend('verwendete Referenzzeit', 'Ideallinie', 'Location', 'best');
title(sprintf('Zeitbasis: %s', cfg.referenceTiming)); grid on;
subplot(3,1,2);
plot(log.t_wall, log.dt_loop, 'LineWidth', 1.2); hold on;
plot(log.t_wall, log.dt_since_send, '--', 'LineWidth', 1.0);
yline(cfg.Ts, 'k:', 'Soll-Ts');
yline(cfg.watchdogTimeout, 'r--', 'Watchdog');
xlabel('t_{wall} [s]'); ylabel('Zeit [s]');
legend('dt_{loop}', 'dt seit letztem Send', 'Location', 'best'); grid on;
subplot(3,1,3);
plot(log.t_wall, log.t_feedback, '-', log.t_wall, log.t_agent, '--');
xlabel('t_{wall} [s]'); ylabel('Dauer [s]');
legend('RefreshFeedback', 'getAction', 'Location', 'best');
title('Wo Zeit verloren geht'); grid on;

figure('Name', 'P2P Deployment: Pipeline-Wirkung');
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

figure('Name', 'P2P Deployment: EE Trajektorie 3D');
plot3(log.ee_pos(:,1), log.ee_pos(:,2), log.ee_pos(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(log.ee_pos(1,1), log.ee_pos(1,2), log.ee_pos(1,3), 'bo', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot3(log.ee_pos(end,1), log.ee_pos(end,2), log.ee_pos(end,3), 'bd', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'b');
plot3(cfg.target_pos(1), cfg.target_pos(2), cfg.target_pos(3), 'gp', ...
      'MarkerSize', 15, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend('EE-Pfad', 'Start', 'Ende', 'Ziel', 'Location', 'best');
title('Point-to-Point Trajektorie (aus URDF-FK)');

% Timing-Diagnose
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
        warning("Loop verpasst %.0f Hz. Siehe Timing-Diagnose-Plot.", cfg.rateHz);
    end
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

function [q_rad, dq_rad, faultFlag] = kinovaReadFullFeedback(apiHandle, cfg)
    % Liest NUR den Joint-Zustand (Position/Velocity) aus RefreshFeedback.
    % Der EE-Zustand wird NICHT mehr aus baseFb (tool_pose/tool_twist)
    % gelesen, sondern getrennt per URDF-FK aus q/dq berechnet
    % (-> computeEEFromFK). baseFb wird nur noch fuer den Fault-Check genutzt.
    [errCode, baseFb, actuatorFb, ~] = ...
        kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errCode ~= 0
        error('RefreshFeedback fehlgeschlagen (errorCode=%d).', errCode);
    end

    % --- Joint-Zustand ---
    q_deg  = extractActuatorField(actuatorFb, 'position', cfg.nJ);
    dq_deg = extractActuatorField(actuatorFb, 'velocity', cfg.nJ);
    % WICHTIG: NICHT mit wrapPi auf (-pi,pi] klemmen! Stattdessen Anchor-
    % relativ entfalten (eindeutig fuer kleine Bewegungen um den Anchor,
    % stimmt mit ungewrappter Trainings-Konvention ueberein).
    q_rad  = unwrapToRef(deg2rad(q_deg(:)), cfg.q_start_anchor(:));
    dq_rad = deg2rad(dq_deg(:));

    faultFlag = checkFaults(actuatorFb, baseFb);
end

function [ee_pos, ee_vel] = computeEEFromFK(robot, q, dq, cfg)
    % EE-Pose/-Geschwindigkeit aus URDF-Forward-Kinematik.
    %   ee_pos : 3x1 EE-Position (URDF-Frame, m)
    %   ee_vel : 3x1 EE-Lineargeschwindigkeit (m/s)
    % q, dq als Spaltenvektoren (nJ x 1). robot.DataFormat == 'row', daher
    % Uebergabe als Zeilenvektor an getTransform/geometricJacobian.
    qRow   = q(:).';
    T      = getTransform(robot, qRow, cfg.eeBodyName);
    ee_pos = T(1:3, 4);
    % geometricJacobian: 6xN -> Zeilen 1:3 = angular, 4:6 = linear (MATLAB)
    J      = geometricJacobian(robot, qRow, cfg.eeBodyName);
    ee_vel = J(4:6, :) * dq(:);
end

function q = unwrapToRef(q_raw, q_ref)
    % Bringt jeden Winkel auf den zur Referenz naechstgelegenen
    % 2*pi-aequivalenten Wert. Stateless, robust gegen +-pi-Grenze.
    q = q_raw + 2*pi * round((q_ref - q_raw) / (2*pi));
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
        [q, ~, faultFlag] = kinovaReadFullFeedback(apiHandle, cfg);
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
    catch, end
    try, kortexApiMexInterface('StopAction', apiHandle); catch, end
    try
        kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
        fprintf('[safeShutdown] MEX-Session geschlossen.\n');
    catch, end
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
    log.t              = zeros(n, 1);   % == t_ref (Kompatibilitaet)
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
    log.ee_pos         = zeros(n, 3);    % URDF-Frame (aus FK)
    log.ee_kortex      = zeros(n, 3);    % == ee_pos (FK), Kompatibilitaet
    log.ee_ref         = zeros(n, 3);    % const target
    log.ep             = zeros(n, 3);
    log.ep_norm        = zeros(n, 1);
end

function log = logRow(log, k, tRef, tWall, dtLoop, dtSinceSend, ...
        tFeedback, tAgent, tSend, overrun, q, dq, dq_raw, dq_cmd, yFilt, ...
        ee_pos_urdf, ref_pos, ep, ee_pos_kortex)
    log.t(k)              = tRef;
    log.t_ref(k)          = tRef;
    log.t_wall(k)         = tWall;
    log.dt_loop(k)        = dtLoop;
    log.dt_since_send(k)  = dtSinceSend;
    log.t_feedback(k)     = tFeedback;
    log.t_agent(k)        = tAgent;
    log.t_send(k)         = tSend;
    log.timing_overrun(k) = overrun;
    log.q(k, :)           = q.';
    log.dq(k, :)          = dq.';
    log.dq_raw(k, :)      = dq_raw.';
    log.dq_cmd(k, :)      = dq_cmd.';
    log.dq_filt(k, :)     = yFilt.';
    log.yFilt(k, :)       = yFilt.';
    log.ee_pos(k, :)      = ee_pos_urdf(:).';
    log.ee_kortex(k, :)   = ee_pos_kortex(:).';
    log.ee_ref(k, :)      = ref_pos.';
    log.ep(k, :)          = ep.';
    log.ep_norm(k)        = norm(ep);
end

function log = truncateLog(log, n)
    if n < 1, return; end
    flds = fieldnames(log);
    for i = 1:numel(flds)
        v = log.(flds{i});
        if isvector(v)
            log.(flds{i}) = v(1:n);
        else
            log.(flds{i}) = v(1:n, :);
        end
    end
end
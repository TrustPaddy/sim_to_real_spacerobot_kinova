%% deploy_agent_kinova.m  (V2 - MEX-Adapter, deg<->rad-Konvertierung)
% Deployt einen trainierten PPO-Agenten auf den echten Kinova Gen3 via
% kortexApiMexInterface (MEX). Repliziert die Simulink-Post-Action-Pipeline
% (Saturation -> Discrete Filter -> Rate Limiter) 1:1 aus dem Trainings-
% Modell und wendet darauf die Hardware-Schutzschicht an.
%
% AENDERUNGEN gegenueber V1:
%   - MEX-Calls an *tatsaechliche* kortexApiMexInterface-API angepasst:
%     CreateRobotApisWrapper / RefreshFeedback / SendJointSpeedCommand /
%     ReachJointAngles / DestroyRobotApisWrapper. apiHandle wird explizit
%     verwaltet.
%   - Einheiten-Konvertierung (MEX in deg/deg-s, Agent/Pipeline in rad/rad-s)
%     in Adapter-Funktionen am Ende der Datei gekapselt.
%   - Feedback-Extraktion robust (Struct vs. Struct-Array, Fault-Felder
%     defensiv per isfield).
%   - Connection-Diagnose beim ersten Feedback (druckt Feldnamen + Werte,
%     damit Abweichungen zur Adapter-Annahme sofort sichtbar werden).
%   - Progressive Scaling via cfg.dqMaxScale.
%
% SICHERHEITSKONZEPT (unveraendert):
%   - Dry-Run Modus (Standard)
%   - Pre-Flight Gate
%   - 29D Obs-Konsistenz-Check
%   - Homing per Position-Control auf IK-loesung
%   - Trainings-Pipeline-Replikation
%   - Per-Joint Velocity-Saettigung auf dq_max
%   - Soft-Limit-Bremsung
%   - Fault-Check, OOD-Stopp bei ||ep|| > 0.4 m
%   - onCleanup: Zero-Velocity + DestroyRobotApisWrapper

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();

% ---- SICHERHEIT: Dry-Run + Pre-Flight ----
cfg.dryRun            = dry;    % true = nur Anzeige, false = echte Kommandos
cfg.preFlightRequired = false;    % nach Simulink-Verifikation auf false setzen

% ---- Agent laden ----
cfg.agentFile      = "";   % leer = neueste .mat in savedAgents_spacekinova/
cfg.expectedObsDim = 29;

% ---- URDF ----
% WARNUNG: SpaceKinova.urdf hat Basis bei z=1.3 m (Montage-Plattform).
% Wenn dein echter Gen3 auf z=0 steht, brauchst du ein Standard-Gen3-URDF
% ODER musst cfg.center entsprechend der realen Montage anpassen. IK und
% FK sind intern konsistent, aber die absoluten z-Werte muessen zum realen
% Aufbau passen.
cfg.urdfFile   = "SpaceKinova.urdf";
cfg.eeBodyName = "kinova_end_effector_link";

% ---- MEX-Interface (kortexApiMexInterface) ----
cfg.kinovaIP         = '192.168.0.10';
cfg.kinovaUser       = 'admin';
cfg.kinovaPassword   = 'admin';
cfg.sessionTimeoutMs = uint32(60000);   % Session-Lifetime
cfg.controlTimeoutMs = uint32(200);     % Kinova stoppt bei Control-Timeout
cfg.speedCmdDuration = 0;                % 0 = kein internes Timeout am Kommando

% ---- Kinova Gen3 Spezifikationen (in rad, rad/s) ----
cfg.nJ         = 7;
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
cfg.dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];

% ---- Action-Skalierung (identisch zum Training, progressives Scaling) ----
cfg.safetyFactor = 0.7;
%cfg.dqMaxScale   = 0.3;     
cfg.dqMaxScale   = 0.2;      % <-- EINSTELLHEBEL: 0.1-0.3 zum Einfahren,
                                %     spaeter bis 1.0 hochdrehen
cfg.dq_max       = cfg.safetyFactor * cfg.dqLim * cfg.dqMaxScale;

% ---- Trainings-Pipeline-Parameter (aus Simulink-Modell) ----
% Saturation: J1,3,5,7 auf 0 geklemmt -> effektive 3-DOF-Steuerung
% Hinweis: die 0.9774 waren fuer dqMaxScale=1. Hier neu skaliert.
baseSatUpper     = [0.0; 0.9774; 0.0; 0.9774; 0.0; 0.1; 0.0];
cfg.satUpper     = baseSatUpper * cfg.dqMaxScale;
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

% ---- Homing ----
cfg.homingSpeed       = deg2rad(10);     % rad/s; wird intern zu deg/s
cfg.homingTolerance   = 0.02;             % m (EE-Position)
cfg.homingWaitTimeout = 20;               % s (max. Wartezeit)
cfg.homingPosTol      = deg2rad(1.0);     % rad (Joint-Toleranz am Ziel)

% ---- Referenztrajektorie (identisch zum Training: URDF-Frame, XZ-Ebene) ----
% WICHTIG: cfg.center MUSS im URDF-Frame bleiben (so wie im Training), NICHT
% im realen Welt-Frame. Der Agent wurde auf Observations trainiert, die
% ueber die URDF-Forward-Kinematik berechnet werden. ep = ref - ee_pos
% muss daher in URDF-Koordinaten bleiben, sonst sieht der Agent einen
% konstanten Pseudo-Fehler in Hoehe des Base-Montage-Offsets.
%
% Der echte EE landet physisch an einer anderen Stelle (verschoben um den
% Offset zwischen realer Base-Montage und URDF-Base). Das ist erwartetes
% Verhalten. Die Gelenkwinkel sind in beiden Frames identisch, nur die
% Welt-Koordinaten des EE unterscheiden sich.
cfg.r      = 0.2;
cfg.center = [0.0, -0.025, 1.687 - cfg.r];   % <- URDF/Training-Frame!
cfg.omega  = pi/cfg.maxDuration;
cfg.yConst = 0;

% ---- Realer Base-Offset (optional, nur fuer Anzeige/Diagnose) ----
% Wenn du weisst, wo deine reale Gen3-Base physisch steht (relativ zum
% URDF-Base-Ursprung), kannst du das hier eintragen. Dann wird vor dem
% Start die *physisch* erwartete EE-Startposition geprintet, damit du
% pruefen kannst, ob der reale Arbeitsraum frei ist.
%
% Beispiel: Wenn du vorher gemessen hast, dass ein real erreichbarer
% Punkt [0.08, -0.017, 1.112] dem URDF-Punkt [0.0, -0.025, 1.487]
% entspricht, dann ist der Offset = real - urdf = [0.08, 0.008, -0.375].
cfg.realBaseOffset = [0.08, 0.008, -0.375];   % [x, y, z] in m; [0 0 0] = deaktiviert

% ---- Observation Limits ----
cfg.ePLim   = 0.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

cfg.oodThreshold = 0.4;

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
    d = dir(fullfile("SavedAgents/MotionProfile/Circle/PPO/SpaceKinova_PPO_agent_motionprofile.mat"));
    if isempty(d)
        error("Kein trainierter Agent gefunden in savedAgents_spacekinova/");
    end
    [~, idx] = max([d.datenum]);
    cfg.agentFile = fullfile(d(idx).folder, d(idx).name);
end

fprintf("Lade Agent: %s\n", cfg.agentFile);
loaded = load(cfg.agentFile);
agent  = loaded.agent;

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
%  4) REFERENZTRAJEKTORIE
%  =========================
t_ref    = 0:cfg.Ts:cfg.maxDuration;
x_ref    = cfg.center(1) + cfg.r * sin(cfg.omega * t_ref);
y_ref    = cfg.center(2) + cfg.yConst * t_ref;
z_ref    = cfg.center(3) + cfg.r * cos(cfg.omega * t_ref);
traj_ref = [x_ref(:) y_ref(:) z_ref(:)];
dt_ref   = mean(diff(t_ref));
vref     = [zeros(1,3); diff(traj_ref)/dt_ref];

%% =========================
%  5) OBSERVATION LIMITS (Clipping)
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
%  5b) REACHABILITY-PREFLIGHT (URDF- und Welt-Frame)
%  =========================
% Sampled die Trajektorie, rechnet IK fuer jeden Punkt und prueft:
%   - Alle Joint-Loesungen innerhalb [qLim_lower, qLim_upper]
%   - Keine unsinnigen Spruenge zwischen aufeinanderfolgenden Punkten
%   - Falls cfg.realBaseOffset != 0: Physische EE-Trajektorie im Welt-Frame
%
% Dient rein zur Verifikation vor Robot-Kontakt. Keine Kommandos.
fprintf("\n== Reachability-Preflight ==\n");
ikCheck     = inverseKinematics('RigidBodyTree', robot_rbt);
weightsChk  = [0.25 0.25 0.25 1 1 1];
nSamples    = 20;
sampleIdx   = round(linspace(1, size(traj_ref, 1), nSamples));
q_seed      = zeros(1, cfg.nJ);
q_prev      = [];
maxJump_rad = deg2rad(45);    % plausibler max. Sprung zwischen 5%-Waypoints

qMin = +inf(1, cfg.nJ);
qMax = -inf(1, cfg.nJ);
reachabilityOk = true;

for s = 1:nSamples
    idx     = sampleIdx(s);
    T_chk   = trvec2tform(traj_ref(idx, :));
    [q_s, info] = ikCheck(char(cfg.eeBodyName), T_chk, weightsChk, q_seed);
    if info.ExitFlag <= 0
        fprintf("  [WARN] IK konvergierte nicht bei idx=%d (t=%.2fs)\n", ...
                idx, t_ref(idx));
        reachabilityOk = false;
    end
    qMin = min(qMin, q_s);
    qMax = max(qMax, q_s);

    % Check Joint-Limits
    viol = (q_s(:) < cfg.qLim_lower) | (q_s(:) > cfg.qLim_upper);
    if any(viol)
        fprintf("  [WARN] Joint-Limit-Verletzung bei idx=%d: J%s\n", ...
                idx, mat2str(find(viol)));
        reachabilityOk = false;
    end

    % Check Spruenge
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

% Physische Start- und End-EE-Position (Welt-Frame) anzeigen
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

    % onCleanup NUR registrieren, wenn Verbindung steht
    cleanupObj = onCleanup(@() safeShutdown(apiHandle, cfg.nJ));  %#ok<NASGU>

    % --- Diagnose-Block: zeigt die tatsaechliche Feedback-Struktur ---
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

    % --- Erstes Feedback ueber den Adapter (jetzt in rad / rad/s) ---
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
%  7) STARTPOSITION ÜBERNEHMEN (Homing übersprungen)
%  =========================
% Roboter steht bereits manuell in Startpose → kein MoveToJoints nötig.
if ~cfg.dryRun
    % Lies aktuelle reale Gelenkwerte (nach manuellem Positionieren)
    [q_now, ~, ~] = kinovaReadFeedback(apiHandle, cfg);
else
    q_now = q0; % Dry-Run: Annahme, dass Start bei q0 liegt
end

% Verifikation: Ist die manuelle Pose nah genug am Trajektorienstart?
T_ee_now  = getTransform(robot_rbt, q_now.', char(cfg.eeBodyName));
err_start = norm(T_ee_now(1:3, 4) - traj_ref(1, :)');

% Da manuell positioniert wurde, nutzen wir eine leichtere Toleranz (5 cm)
% und werfen bei Abweichung nur eine Warnung statt eines harten Abbruchs.
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
fprintf(" - dqMaxScale = %.2f (effektives dq_max = [%s] rad/s)\n", ...
        cfg.dqMaxScale, join(string(round(cfg.dq_max, 3)), ", "));
fprintf(" - Laufzeit: %.1f s\n", cfg.maxDuration);
fprintf(" - Trajektorie wird im URDF-Frame gerechnet (= Training-Frame).\n");
fprintf("   Der reale EE faehrt physisch um cfg.realBaseOffset = [%s] verschoben.\n", ...
        join(string(cfg.realBaseOffset), " "));
fprintf(" - Dry-Run: %s\n\n", string(cfg.dryRun));

if ~cfg.dryRun
    input("ENTER zum Starten (CTRL+C zum Abbrechen)... ", "s");
end

%% =========================
%  9) PIPELINE-STATE
%  =========================
state = struct();
state.yFilt  = zeros(cfg.nJ, 1);
state.dqPrev = zeros(cfg.nJ, 1);

%% =========================
%  10) HAUPTSCHLEIFE
%  =========================
r = rateControl(cfg.rateHz);
nSteps = numel(t_ref);

log = initLog(nSteps, cfg.nJ);

q  = q_now;
dq = dq0;

fprintf("== Starte Deployment-Loop (%d Schritte, %.1f Hz) ==\n", ...
        nSteps, cfg.rateHz);

k_end = 0;
for k = 1:nSteps
    tNow = t_ref(k);

    % --- Zustand lesen + Fault-Check ---
    if ~cfg.dryRun
        [q, dq, faultFlag] = kinovaReadFeedback(apiHandle, cfg);
        if faultFlag
            fprintf('\n[FAULT] Kinova-Fault erkannt -- Stopp\n');
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
            k_end = k;
            break;
        end
    end

    % --- FK + Jacobian ---
    T_ee   = getTransform(robot_rbt, q.', char(cfg.eeBodyName));
    ee_pos = T_ee(1:3, 4);
    ee_rot = T_ee(1:3, 1:3);
    J_ee   = geometricJacobian(robot_rbt, q.', char(cfg.eeBodyName));

    % --- Referenz + Fehler ---
    ref_pos = traj_ref(k, :).';
    ref_vel = vref(k, :).';
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

    % --- OOD-Check ---
    if norm(ep) > cfg.oodThreshold
        fprintf(['\n[STOPP] Positionsfehler %.3f m > Schwelle %.3f m ' ...
                 '(Out-of-Distribution)\n'], norm(ep), cfg.oodThreshold);
        if ~cfg.dryRun
            kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
        end
        k_end = k;
        break;
    end

    % --- Obs-Clipping ---
    obs = max(min(obs, obsHigh), obsLow);

    % --- Agent abfragen ---
    action = getAction(agent, {obs});
    dq_raw = action{1}(:);

    % --- Action-Skalierung ---
    dq_scaled = dq_raw .* cfg.dq_max;

    % --- Pipeline (Sat -> Filter -> RateLim) ---
    [dq_cmd, state] = applyTrainingPipeline(dq_scaled, state, cfg);

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

    % --- HW-Schutz: Saettigung auf dq_max ---
    dq_cmd = max(min(dq_cmd, cfg.dq_max), -cfg.dq_max);

    % --- Senden / Anzeigen ---
    if cfg.dryRun
        if mod(k, cfg.rateHz) == 1
            fprintf("t=%.2fs | ep=[%.3f %.3f %.3f]m | dq_cmd=[%s] rad/s\n", ...
                tNow, ep(1), ep(2), ep(3), ...
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

    % --- Logging ---
    log.t(k)           = tNow;
    log.q(k, :)        = q.';
    log.dq(k, :)       = dq.';
    log.dq_raw(k, :)   = dq_raw.';
    log.dq_scaled(k, :) = dq_scaled.';
    log.dq_cmd(k, :)   = dq_cmd.';
    log.yFilt(k, :)    = state.yFilt.';
    log.ee_pos(k, :)   = ee_pos.';
    log.ee_ref(k, :)   = ref_pos.';
    log.ep(k, :)       = ep.';

    if ~cfg.dryRun
        waitfor(r);
    end
    k_end = k;
end

% --- Final Stop ---
if ~cfg.dryRun
    kinovaSendVelocity(apiHandle, zeros(cfg.nJ, 1), cfg);
end

fprintf("\n== Deployment beendet (k=%d/%d) ==\n", k_end, nSteps);

%% =========================
%  11) LOG + AUSWERTUNG
%  =========================
log = truncateLog(log, k_end);

ep_norms = vecnorm(log.ep, 2, 2);
fprintf("Position Tracking RMSE: %.4f m\n", rms(ep_norms));
fprintf("Position Tracking Max:  %.4f m\n", max(ep_norms));

logFile = sprintf("deploy_log_%s.mat", ...
                  string(datetime('now'), 'yyyyMMdd_HHmmss'));
save(logFile, "cfg", "log");
fprintf("Log gespeichert: %s\n", logFile);

figure('Name', 'Deployment: EE Tracking');
subplot(2,1,1);
plot(log.t, log.ee_ref, '--', log.t, log.ee_pos, '-');
legend('ref_x','ref_y','ref_z','x','y','z');
xlabel('t [s]'); ylabel('Position [m]'); title('End-Effector Tracking'); grid on;
subplot(2,1,2);
plot(log.t, ep_norms);
xlabel('t [s]'); ylabel('||ep|| [m]'); title('Positionsfehler'); grid on;

figure('Name', 'Deployment: Pipeline-Wirkung');
subplot(3,1,1);
plot(log.t, log.dq_raw);
xlabel('t [s]'); ylabel('dq_{raw} [-1..1]'); title('Agent-Action (normiert)');
legend(arrayfun(@(j) sprintf("J%d", j), 1:cfg.nJ, 'UniformOutput', false));
grid on;
subplot(3,1,2);
plot(log.t, log.dq_scaled);
xlabel('t [s]'); ylabel('dq_{scaled} [rad/s]');
title('Nach Skalierung (dq\_raw * dq\_max)');
grid on;
subplot(3,1,3);
plot(log.t, log.dq_cmd);
xlabel('t [s]'); ylabel('dq_{cmd} [rad/s]');
title('Nach Pipeline (Sat + Filter + RateLim + HW-Safety)');
grid on;


%% ============ MEX-ADAPTER-FUNKTIONEN ============
% Diese Funktionen kapseln *alle* MEX-Calls und die deg<->rad-Umrechnung.
% Main-Code bleibt dadurch durchgaengig in rad/rad/s.

function apiHandle = kinovaOpen(cfg)
% Baut die MEX-Session auf und gibt das apiHandle zurueck.
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

function [q_rad, dq_rad, faultFlag] = kinovaReadFeedback(apiHandle, cfg)
% Liest Feedback und konvertiert deg -> rad.
% faultFlag: true falls irgendein Fault-Register != 0 (defensiv gepr.).
    [errCode, baseFb, actuatorFb, ~] = ...
        kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errCode ~= 0
        error('RefreshFeedback fehlgeschlagen (errorCode=%d).', errCode);
    end

    % Positionen (Grad) und Geschwindigkeiten (deg/s) robust extrahieren
    q_deg  = extractActuatorField(actuatorFb, 'position',  cfg.nJ);
    dq_deg = extractActuatorField(actuatorFb, 'velocity',  cfg.nJ);

    q_rad  = deg2rad(q_deg(:));
    dq_rad = deg2rad(dq_deg(:));

    % Fault-Check: defensiv - es sind mehrere Namen in Kinova-Versionen
    % ueblich. Wir probieren der Reihe nach.
    faultFlag = checkFaults(actuatorFb, baseFb);
end

function errCode = kinovaSendVelocity(apiHandle, dq_rad, cfg)
% Sendet Geschwindigkeitskommando. Konvertiert rad/s -> deg/s.
    dq_deg = rad2deg(dq_rad(:)).';   % als Zeilenvektor
    errCode = kortexApiMexInterface('SendJointSpeedCommand', ...
        apiHandle, cfg.speedCmdDuration, dq_deg, uint32(cfg.nJ));
end

function kinovaMoveToJoints(apiHandle, q_rad, cfg)
% Homing-Kommando ueber ReachJointAngles mit Speed-Constraint.
% Nimmt rad entgegen, sendet deg.
    q_deg = rad2deg(q_rad(:)).';
    homingSpeed_deg = rad2deg(cfg.homingSpeed);

    % Erst mit Speed-Constraint versuchen
    errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
        int32(2), homingSpeed_deg, 0, q_deg);

    if errCode ~= 0
        % Fallback: Default-Speed (wie im Velocity-Test erfolgreich)
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
% Wartet bis alle Joints innerhalb cfg.homingPosTol und stabil bleiben.
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
% onCleanup: Zero-Velocity + Session-Close. Best-effort.
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
% Robuste Extraktion eines per-Joint-Feldes aus actuatorFb.
% Funktioniert fuer:
%   a) einzelner Struct mit Feld als Vektor (Laenge >= nJ)
%   b) Struct-Array (1 Element pro Joint, skalares Feld)
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
% Defensiver Fault-Check. Prueft in dieser Reihenfolge:
%   - actuatorFb(i).fault_bank_a/b  (per-Joint)
%   - actuatorFb(i).status_flags
%   - baseFb.fault_bank_a/b         (global)
    flag = false;

    % Per-Joint Fault-Bank
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

    % Globale Fault-Felder in baseFb
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
% Wrap auf [-pi, pi]
    wrapped = mod(angle + pi, 2*pi) - pi;
end

function [dq_out, state] = applyTrainingPipeline(dq_raw, state, cfg)
% Saturation -> IIR-Filter 1. Ordnung -> Rate-Limiter. Identisch zum
% Simulink-Trainingspfad.
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
    log.t         = zeros(n, 1);
    log.q         = zeros(n, nJ);
    log.dq        = zeros(n, nJ);
    log.dq_raw    = zeros(n, nJ);
    log.dq_scaled = zeros(n, nJ);
    log.dq_cmd    = zeros(n, nJ);
    log.yFilt     = zeros(n, nJ);
    log.ee_pos    = zeros(n, 3);
    log.ee_ref    = zeros(n, 3);
    log.ep        = zeros(n, 3);
end

function log = truncateLog(log, n)
    if n < 1, return; end
    log.t         = log.t(1:n);
    log.q         = log.q(1:n, :);
    log.dq        = log.dq(1:n, :);
    log.dq_raw    = log.dq_raw(1:n, :);
    log.dq_scaled = log.dq_scaled(1:n, :);
    log.dq_cmd    = log.dq_cmd(1:n, :);
    log.yFilt     = log.yFilt(1:n, :);
    log.ee_pos    = log.ee_pos(1:n, :);
    log.ee_ref    = log.ee_ref(1:n, :);
    log.ep        = log.ep(1:n, :);
end
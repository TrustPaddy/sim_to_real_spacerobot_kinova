%% deploy_agent_kinova.m
% Deployt einen trainierten PPO-Agenten auf den echten Kinova Gen3 via ROS.
%
% SICHERHEITSKONZEPT:
%   - Dry-Run Modus (Standard): Kommandos werden nur angezeigt, NICHT gesendet
%   - Per-Joint Velocity-Saettigung bei 70% der Hardware-Limits
%   - Gelenkpositions-Ueberwachung mit Soft-Limit Warnung
%   - Watchdog: Kein joint_state -> sofortiger Stopp
%   - onCleanup: Sendet immer Zero-Velocity bei Abbruch
%   - Out-of-Distribution Erkennung: Stopp bei extremen Beobachtungen
%
% ABLAUF:
%   1. cfg.dryRun = true setzen (Standard)
%   2. Skript ausfuehren, Ausgaben pruefen
%   3. cfg.dryRun = false setzen fuer echte Ausfuehrung
%
% Konfiguriert fuer: Kinova Gen3 7-DOF, kortex_driver (ROS Noetic)

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();

% ---- SICHERHEIT: Dry-Run Modus ----
cfg.dryRun = true;   % true = nur Anzeige, false = echte Kommandos
% WARNUNG: Erst auf false setzen, nachdem du die Dry-Run Ausgaben verifiziert hast!

% ---- Agent laden ----
cfg.agentFile = "";   % z.B. "savedAgents_spacekinova/ppo_spacekinova_vel_20240101_120000.mat"
% Falls leer: sucht neueste .mat Datei in savedAgents_spacekinova/

% ---- URDF ----
cfg.urdfFile   = "SpaceKinova.urdf";
cfg.eeBodyName = "kinova_end_effector_link";

% ---- ROS Konfiguration (Kinova Gen3, kortex_driver Noetic) ----
cfg.rosMasterURI = "http://192.168.1.10:11311"; % Anpassen!
cfg.rosNodeIP    = "";
cfg.ns           = "/my_gen3";
cfg.stateTopic   = "/joint_states";
cfg.cmdTopic     = cfg.ns + "/in/joint_velocity";
cfg.cmdMsgType   = "kortex_driver/Base_JointSpeeds";
cfg.jointNames   = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"];

% ---- Kinova Gen3 Spezifikationen ----
cfg.nJ = 7;

% Positionslimits [rad]
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];

% Geschwindigkeitslimits [rad/s]
cfg.dqLim = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];

% Deployment-Limits: 70% der Hardware-Limits (wie im Training)
cfg.safetyFactor = 0.7;
cfg.dq_max = cfg.safetyFactor * cfg.dqLim;

% Soft-Limit Warnung: Abstand zu Gelenklimit [rad] ab dem gebremst wird
cfg.qSoftMargin = deg2rad(10);  % 10 Grad vor Limit -> Geschwindigkeit reduzieren

% ---- Steuerung ----
cfg.rateHz         = 40;     % 40 Hz = Kinova High-Level Servo Rate
cfg.maxDuration    = 8.5;    % Maximale Laufzeit [s]
cfg.watchdogTimeout = 0.10;  % 100 ms Watchdog

% ---- Referenztrajektorie (muss identisch zum Training sein!) ----
cfg.r      = 0.4;
cfg.center = [4.5 - cfg.r, 0.0, 0.0];
cfg.omega  = pi/cfg.maxDuration;
cfg.zConst = 0.0;

% ---- Observation Limits (identisch zum Training) ----
cfg.ePLim   = 0.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

% OOD-Schwelle: Wenn Positionsfehler > diesen Wert -> Stopp
cfg.oodThreshold = 0.4; % [m] - groesser als im Training normal

%% =========================
%  1) AGENT LADEN
%  =========================
if strlength(cfg.agentFile) == 0
    % Neueste .mat Datei suchen
    d = dir(fullfile("savedAgents_spacekinova", "**", "*.mat"));
    if isempty(d)
        error("Kein trainierter Agent gefunden in savedAgents_spacekinova/");
    end
    [~, idx] = max([d.datenum]);
    cfg.agentFile = fullfile(d(idx).folder, d(idx).name);
end

fprintf("Lade Agent: %s\n", cfg.agentFile);
loaded = load(cfg.agentFile);
agent = loaded.agent;

%% =========================
%  2) ROBOT MODELL LADEN
%  =========================
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'column';

%% =========================
%  3) REFERENZTRAJEKTORIE
%  =========================
Ts = 1/cfg.rateHz;
t_ref = 0:Ts:cfg.maxDuration;
x_ref = cfg.center(1) + cfg.r*cos(cfg.omega*t_ref);
y_ref = cfg.center(2) + cfg.r*sin(cfg.omega*t_ref);
z_ref = cfg.center(3) + cfg.zConst*t_ref;
traj_ref = [x_ref(:) y_ref(:) z_ref(:)];

dt_ref = mean(diff(t_ref));
vref = [zeros(1,3); diff(traj_ref)/dt_ref];

%% =========================
%  4) OBSERVATION LIMITS (fuer Clipping)
%  =========================
obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
    cfg.qLim_lower; ...
    -cfg.dqLim; ...
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ...
    ];
obsHigh = [ ...
    cfg.ePLim   * ones(3,1); ...
    cfg.eVLim   * ones(3,1); ...
    cfg.qLim_upper; ...
    cfg.dqLim; ...
    cfg.vBLim   * ones(3,1); ...
    cfg.wBLim   * ones(3,1); ...
    cfg.eOriLim * ones(3,1) ...
    ];

%% =========================
%  5) ROS VERBINDUNG
%  =========================
if ~cfg.dryRun
    fprintf("== ROS init ==\n");
    try
        rosnode list;
        fprintf("ROS bereits initialisiert.\n");
    catch
        if strlength(cfg.rosMasterURI) > 0
            rosinit(cfg.rosMasterURI);
        else
            rosinit;
        end
    end

    cleanupObj = onCleanup(@()safeShutdown(cfg)); %#ok<NASGU>

    stateSub = rossubscriber(cfg.stateTopic, "sensor_msgs/JointState");
    cmdPub   = rospublisher(cfg.cmdTopic, cfg.cmdMsgType);

    % Warte auf ersten joint_state
    fprintf("Warte auf joint_states...\n");
    [q0, dq0, ~] = waitForJointState(stateSub, cfg.jointNames, cfg.watchdogTimeout * 5);
    fprintf("Verbindung OK. q0 = [%s]\n", join(string(round(q0,3)), ", "));
else
    fprintf("\n=== DRY-RUN MODUS ===\n");
    fprintf("Keine ROS-Verbindung. Kommandos werden nur angezeigt.\n\n");
    % Simulierte Startposition (Home)
    q0  = zeros(cfg.nJ, 1);
    dq0 = zeros(cfg.nJ, 1);
end

%% =========================
%  6) SICHERHEITSHINWEIS
%  =========================
fprintf("\n== SICHERHEITSHINWEIS ==\n");
fprintf(" - Arbeitsraum frei, E-Stop bereit\n");
fprintf(" - Max. Geschwindigkeit: [%s] rad/s\n", join(string(round(cfg.dq_max,3)), ", "));
fprintf(" - Laufzeit: %.1f s\n", cfg.maxDuration);
fprintf(" - Dry-Run: %s\n\n", string(cfg.dryRun));

if ~cfg.dryRun
    input("ENTER zum Starten (CTRL+C zum Abbrechen)... ", "s");
end

%% =========================
%  7) HAUPTSCHLEIFE
%  =========================
r = rateControl(cfg.rateHz);
nSteps = numel(t_ref);

% Log-Strukturen
log.t      = zeros(nSteps, 1);
log.q      = zeros(nSteps, cfg.nJ);
log.dq     = zeros(nSteps, cfg.nJ);
log.dq_cmd = zeros(nSteps, cfg.nJ);
log.ee_pos = zeros(nSteps, 3);
log.ee_ref = zeros(nSteps, 3);
log.ep     = zeros(nSteps, 3);

q  = q0;
dq = dq0;

fprintf("== Starte Deployment-Loop (%d Schritte, %.1f Hz) ==\n", nSteps, cfg.rateHz);

for k = 1:nSteps
    tNow = t_ref(k);

    % --- Zustand lesen ---
    if ~cfg.dryRun
        [q, dq, ~] = waitForJointState(stateSub, cfg.jointNames, cfg.watchdogTimeout);
    end

    % --- Forward Kinematics: aktuelle EE-Position ---
    T_ee = getTransform(robot_rbt, q, char(cfg.eeBodyName));
    ee_pos = T_ee(1:3, 4);
    ee_rot = T_ee(1:3, 1:3);

    % --- Referenz fuer diesen Zeitschritt ---
    ref_pos = traj_ref(k, :).';
    ref_vel = vref(k, :).';

    % --- Observation berechnen (29D, identisch zum Training) ---
    % Positionsfehler
    ep = ref_pos - ee_pos;

    % Geschwindigkeitsfehler (ueber Jacobian)
    J_ee = geometricJacobian(robot_rbt, q, char(cfg.eeBodyName));
    ee_vel = J_ee(4:6, :) * dq;  % Translationsgeschwindigkeit (Zeilen 4-6)
    ev = ref_vel - ee_vel;

    % Orientierungsfehler (gegen Identitaet)
    R_err = eye(3) * ee_rot.';
    e_ori = [R_err(3,2) - R_err(2,3); R_err(1,3) - R_err(3,1); R_err(2,1) - R_err(1,2)] / 2;

    % Basis-Geschwindigkeit: Auf echtem Kinova ist Basis fest -> Null
    v_base = zeros(3,1);
    w_base = zeros(3,1);

    % Observation zusammenbauen
    obs = [ep; ev; q; dq; v_base; w_base; e_ori];

    % --- OOD-Check: Positionsfehler zu gross -> Stopp ---
    if norm(ep) > cfg.oodThreshold
        fprintf("\n[STOPP] Positionsfehler %.3f m > Schwelle %.3f m (Out-of-Distribution)\n", ...
            norm(ep), cfg.oodThreshold);
        if ~cfg.dryRun
            sendJointVelocity(cmdPub, cfg, zeros(cfg.nJ, 1));
        end
        break;
    end

    % --- Observation clippen (wie im Training) ---
    obs = max(min(obs, obsHigh), obsLow);

    % --- Agent abfragen ---
    action = getAction(agent, {obs});
    dq_cmd = action{1};
    dq_cmd = dq_cmd(:);

    % --- Sicherheits-Saettigung (per-Joint) ---
    dq_cmd = max(min(dq_cmd, cfg.dq_max), -cfg.dq_max);

    % --- Soft-Limit Bremsung: nahe an Gelenklimit -> Geschwindigkeit reduzieren ---
    for j = 1:cfg.nJ
        distToUpper = cfg.qLim_upper(j) - q(j);
        distToLower = q(j) - cfg.qLim_lower(j);

        if distToUpper < cfg.qSoftMargin && dq_cmd(j) > 0
            scale = max(distToUpper / cfg.qSoftMargin, 0);
            dq_cmd(j) = dq_cmd(j) * scale;
        end
        if distToLower < cfg.qSoftMargin && dq_cmd(j) < 0
            scale = max(distToLower / cfg.qSoftMargin, 0);
            dq_cmd(j) = dq_cmd(j) * scale;
        end
    end

    % --- Kommando senden oder anzeigen ---
    if cfg.dryRun
        if mod(k, cfg.rateHz) == 1  % einmal pro Sekunde anzeigen
            fprintf("t=%.2fs | ep=[%.3f %.3f %.3f]m | dq_cmd=[%s] rad/s\n", ...
                tNow, ep(1), ep(2), ep(3), ...
                join(string(round(dq_cmd,4)), " "));
        end
        % Im Dry-Run: q/dq bleiben konstant (keine Physik-Simulation)
    else
        sendJointVelocity(cmdPub, cfg, dq_cmd);
    end

    % --- Logging ---
    log.t(k)        = tNow;
    log.q(k,:)      = q.';
    log.dq(k,:)     = dq.';
    log.dq_cmd(k,:) = dq_cmd.';
    log.ee_pos(k,:) = ee_pos.';
    log.ee_ref(k,:) = ref_pos.';
    log.ep(k,:)     = ep.';

    if ~cfg.dryRun
        waitfor(r);
    end
end

% --- Final Stop ---
if ~cfg.dryRun
    sendJointVelocity(cmdPub, cfg, zeros(cfg.nJ, 1));
end

fprintf("\n== Deployment beendet (k=%d/%d) ==\n", min(k, nSteps), nSteps);

%% =========================
%  8) LOG SPEICHERN + AUSWERTUNG
%  =========================
log.t      = log.t(1:min(k,nSteps));
log.q      = log.q(1:min(k,nSteps), :);
log.dq     = log.dq(1:min(k,nSteps), :);
log.dq_cmd = log.dq_cmd(1:min(k,nSteps), :);
log.ee_pos = log.ee_pos(1:min(k,nSteps), :);
log.ee_ref = log.ee_ref(1:min(k,nSteps), :);
log.ep     = log.ep(1:min(k,nSteps), :);

% Tracking-Metriken
ep_norms = vecnorm(log.ep, 2, 2);
fprintf("Position Tracking RMSE: %.4f m\n", rms(ep_norms));
fprintf("Position Tracking Max:  %.4f m\n", max(ep_norms));

% Speichern
logFile = sprintf("deploy_log_%s.mat", datestr(now, 'yyyymmdd_HHMMSS'));
save(logFile, "cfg", "log");
fprintf("Log gespeichert: %s\n", logFile);

% Plot
figure('Name', 'Deployment: EE Tracking');
subplot(2,1,1);
plot(log.t, log.ee_ref, '--', log.t, log.ee_pos, '-');
legend('ref_x','ref_y','ref_z','x','y','z');
xlabel('t [s]'); ylabel('Position [m]'); title('End-Effector Tracking');
grid on;

subplot(2,1,2);
plot(log.t, ep_norms);
xlabel('t [s]'); ylabel('||ep|| [m]'); title('Positionsfehler');
grid on;

figure('Name', 'Deployment: Joint Velocities');
plot(log.t, log.dq_cmd);
xlabel('t [s]'); ylabel('dq\_cmd [rad/s]'); title('Kommandierte Gelenkgeschwindigkeiten');
legend(arrayfun(@(j) sprintf("J%d", j), 1:cfg.nJ, 'UniformOutput', false));
grid on;

%% ============ LOKALE FUNKTIONEN ============

function safeShutdown(cfg)
    try
        fprintf("\n[safeShutdown] Sende Zero-Command...\n");
        pub = rospublisher(cfg.cmdTopic, cfg.cmdMsgType);
        sendJointVelocity(pub, cfg, zeros(cfg.nJ, 1));
        pause(0.1);
    catch
    end
    try
        rosshutdown;
        fprintf("[safeShutdown] ROS shutdown.\n");
    catch
    end
end

function sendJointVelocity(pub, cfg, dq_cmd)
    msg = rosmessage(pub);
    nJ = numel(dq_cmd);
    msg.JointSpeeds = repmat(msg.JointSpeeds, 1, nJ);
    for i = 1:nJ
        msg.JointSpeeds(i).JointIdentifier = uint32(i-1); % 0-basiert
        msg.JointSpeeds(i).Value          = double(dq_cmd(i));
        msg.JointSpeeds(i).Duration       = 0;
    end
    send(pub, msg);
end

function [q, dq, tstamp] = waitForJointState(stateSub, jointNames, timeout_s)
    msg = receive(stateSub, timeout_s);
    if isempty(msg)
        error("Kein joint_state innerhalb %.0f ms empfangen.", timeout_s*1000);
    end
    try
        tstamp = double(msg.Header.Stamp.Sec) + 1e-9*double(msg.Header.Stamp.Nsec);
    catch
        tstamp = now*86400;
    end
    names = string(msg.Name);
    nJ = numel(jointNames);
    q  = zeros(nJ, 1);
    dq = zeros(nJ, 1);
    for i = 1:nJ
        idx = find(names == jointNames(i), 1, "first");
        if isempty(idx)
            error("Joint '%s' nicht in /joint_states gefunden.", jointNames(i));
        end
        if numel(msg.Position) >= idx
            q(i) = double(msg.Position(idx));
        end
        if numel(msg.Velocity) >= idx
            dq(i) = double(msg.Velocity(idx));
        end
    end
end

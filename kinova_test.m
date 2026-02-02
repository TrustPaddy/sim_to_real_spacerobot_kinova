%% kinova_step1_velocity_test.m
% Schritt 1: Sicherer Velocity-Jog Test für Kinova via ROS (z.B. kortex_driver)
% - Liest joint_states
% - Sendet dq_cmd (Gelenkgeschwindigkeit)
% - Safety: Saturation, Watchdog, onCleanup -> stoppt sicher
% - Logging: t, q_meas, dq_meas, dq_cmd

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg.backend            = "ros1";          % aktuell nur ROS1 implementiert
cfg.rosMasterURI       = "";              % z.B. "http://192.168.1.2:11311" oder "" für lokal
cfg.rosNodeIP          = "";              % z.B. "192.168.1.100" oder "" für auto

cfg.ns                 = "/my_gen3";      % Namespace deines Roboters (häufig /my_gen3)
cfg.stateTopic         = "/joint_states"; % oft global: /joint_states
% Velocity command topic (häufig bei kortex_driver):
cfg.cmdTopic           = cfg.ns + "/in/joint_speeds"; % Alternative: "/in/joint_velocity" o.ä.

% Message type für Velocity-Kommandos:
% - kortex_driver/JointSpeeds  (häufig)
% - trajectory_msgs/JointTrajectory (Fallback, wenn Controller das akzeptiert)
cfg.cmdMsgTypePrimary  = "kortex_driver/JointSpeeds";
cfg.cmdMsgTypeFallback = "trajectory_msgs/JointTrajectory";

% Joint-Namen (Reihenfolge muss zu /joint_states passen).
% Für Gen3 (7DoF) oft: joint_1 ... joint_7
cfg.jointNames = ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6","joint_7"];

% Sicherheitslimits & Testprofil
cfg.rateHz            = 100;       % Sende-/Log-Rate
cfg.stateTimeout_s    = 0.25;      % wenn so lange kein joint_state: stoppe
cfg.dqMax_abs         = 0.25;      % rad/s (sehr konservativ). Falls dein Driver deg/s erwartet: ist noch sicherer.
cfg.testSpeed         = 0.10;      % rad/s (oder deg/s je nach Driver) -> klein halten
cfg.testDuration_s    = 1.5;       % pro Joint
cfg.restDuration_s    = 1.0;       % Pause/Stop zwischen Tests
cfg.settleDuration_s  = 0.5;       % initiales Settling

cfg.saveLog           = true;
cfg.logFile           = "kinova_step1_velocity_log.mat";

% Optional: Pro Joint in beide Richtungen testen
cfg.testBothDirections = true;  % true: +speed und -speed

%% =========================
%  ROS INITIALISIEREN
%  =========================
fprintf("== ROS init ==\n");
try
    rosnode list; % test if already initialized
    fprintf("ROS scheint bereits initialisiert.\n");
catch
    % not initialized -> rosinit
    if cfg.rosMasterURI ~= ""
        if cfg.rosNodeIP ~= ""
            rosinit(cfg.rosMasterURI, "NodeHost", cfg.rosNodeIP);
        else
            rosinit(cfg.rosMasterURI);
        end
    else
        if cfg.rosNodeIP ~= ""
            rosinit("NodeHost", cfg.rosNodeIP);
        else
            rosinit;
        end
    end
end

cleanupObj = onCleanup(@()safeShutdown(cfg)); %#ok<NASGU>

%% =========================
%  SUBSCRIBER / PUBLISHER
%  =========================
fprintf("== Subscriber/Pub setup ==\n");
stateSub = rossubscriber(cfg.stateTopic, "sensor_msgs/JointState");

% Publisher mit Primary msg type versuchen, sonst fallback
[cmdPub, cmdMsgType] = makeVelocityPublisher(cfg);

fprintf("Command publisher: topic=%s, msgType=%s\n", cfg.cmdTopic, cmdMsgType);

%% =========================
%  INITIAL: Zustand holen
%  =========================
fprintf("== Warte auf joint_states ... ==\n");
[q0, dq0, t0] = waitForJointState(stateSub, cfg.jointNames, cfg.stateTimeout_s);
fprintf("Empfangen: t=%.3f s, |q|=%.3f, |dq|=%.3f\n", t0, norm(q0), norm(dq0));

% Kurze Beruhigung
pause(cfg.settleDuration_s);

%% =========================
%  TEST-SEQUENZ
%  =========================
nJ = numel(cfg.jointNames);
r  = rateControl(cfg.rateHz);

% Log-Strukturen (prealloc grob)
log.t      = [];
log.q      = [];
log.dq     = [];
log.dq_cmd = [];
log.note   = {};

fprintf("\n== SICHERHEITSHINWEIS ==\n");
fprintf(" - Arbeitsraum frei, E-Stop bereit.\n");
fprintf(" - Das Skript sendet sehr kleine dq-Kommandos.\n");
fprintf(" - Abbruch mit CTRL+C stoppt durch onCleanup automatisch (Zero command).\n\n");

input("Druecke ENTER, um den Test zu starten... ", "s");

% Hilfsvektor
dq_cmd = zeros(nJ,1);

for j = 1:nJ
    fprintf("\n--- Joint %d / %d (%s) ---\n", j, nJ, cfg.jointNames(j));

    dirs = +1;
    if cfg.testBothDirections
        dirs = [+1, -1];
    end

    for d = dirs
        fprintf("Richtung: %s\n", ternary(d>0, "+", "-"));
        input("ENTER zum Ausfuehren (oder CTRL+C zum Abbrechen)... ", "s");

        dq_cmd(:) = 0;
        dq_cmd(j) = d * cfg.testSpeed;

        % Sättigung
        dq_cmd = saturate(dq_cmd, cfg.dqMax_abs);

        % Für definierte Dauer senden
        tStart = tic;
        while toc(tStart) < cfg.testDuration_s
            % Zustand lesen + Watchdog
            [q, dq, tstamp] = waitForJointState(stateSub, cfg.jointNames, cfg.stateTimeout_s);

            % Kommando senden
            sendJointVelocity(cmdPub, cmdMsgType, cfg, dq_cmd);

            % Log
            log.t(end+1,1)        = tstamp; %#ok<SAGROW>
            log.q(end+1,:)        = q.';     %#ok<SAGROW>
            log.dq(end+1,:)       = dq.';    %#ok<SAGROW>
            log.dq_cmd(end+1,:)   = dq_cmd.'; %#ok<SAGROW>
            log.note{end+1,1}     = sprintf("joint=%d dir=%+d", j, d); %#ok<SAGROW>

            waitfor(r);
        end

        % Stop-Phase
        dq_cmd(:) = 0;
        tStop = tic;
        while toc(tStop) < cfg.restDuration_s
            % optional state lesen, aber vor allem: 0 senden
            sendJointVelocity(cmdPub, cmdMsgType, cfg, dq_cmd);

            [q, dq, tstamp] = waitForJointState(stateSub, cfg.jointNames, cfg.stateTimeout_s);

            log.t(end+1,1)        = tstamp; %#ok<SAGROW>
            log.q(end+1,:)        = q.';     %#ok<SAGROW>
            log.dq(end+1,:)       = dq.';    %#ok<SAGROW>
            log.dq_cmd(end+1,:)   = dq_cmd.'; %#ok<SAGROW>
            log.note{end+1,1}     = sprintf("joint=%d STOP", j); %#ok<SAGROW>

            waitfor(r);
        end

        fprintf("OK: Joint %d Richtung %s abgeschlossen.\n", j, ternary(d>0, "+", "-"));
    end
end

% Final stop
sendJointVelocity(cmdPub, cmdMsgType, cfg, zeros(nJ,1));
fprintf("\n== Test fertig. Robot stopped. ==\n");

%% =========================
%  LOG SPEICHERN + QUICKPLOT
%  =========================
if cfg.saveLog
    save(cfg.logFile, "cfg", "log");
    fprintf("Log gespeichert: %s\n", cfg.logFile);
end

% Quick plot
try
    figure; 
    plot(log.t - log.t(1), log.dq_cmd);
    grid on; xlabel("t [s]"); ylabel("dq\_cmd");
    title("Commanded Joint Velocities");
    legend(cfg.jointNames, "Interpreter", "none");
catch
end

%% ============ LOKALE FUNKTIONEN ============

function safeShutdown(cfg)
% Wird bei Script-Ende oder CTRL+C ausgeführt
    try
        fprintf("\n[safeShutdown] Sende Zero-Command...\n");
        % ROS publisher ggf. nicht mehr im scope -> best effort: rospublisher neu
        [cmdPub, msgType] = makeVelocityPublisher(cfg);
        nJ = numel(cfg.jointNames);
        sendJointVelocity(cmdPub, msgType, cfg, zeros(nJ,1));
        pause(0.1);
    catch
    end
    try
        rosshutdown;
        fprintf("[safeShutdown] ROS shutdown.\n");
    catch
    end
end

function [pub, msgType] = makeVelocityPublisher(cfg)
% Erst Primary versuchen, dann Fallback
    msgType = cfg.cmdMsgTypePrimary;
    try
        pub = rospublisher(cfg.cmdTopic, msgType);
        % test create message
        rosmessage(pub);
        return;
    catch
        % fallback
        msgType = cfg.cmdMsgTypeFallback;
        pub = rospublisher(cfg.cmdTopic, msgType);
        rosmessage(pub);
    end
end

function sendJointVelocity(pub, msgType, cfg, dq_cmd)
% Sendet dq_cmd über gewählten Message-Typ
    dq_cmd = dq_cmd(:);

    switch string(msgType)
        case "kortex_driver/JointSpeeds"
            % Erwartete Struktur (typisch Kortex ROS):
            % msg.joint_speeds(i).joint_identifier = i-1 oder i (je nach driver)
            % msg.joint_speeds(i).value           = velocity
            % msg.joint_speeds(i).duration        = 0
            msg = rosmessage(pub);

            % Einige Treiber nutzen 0-basierte Identifiers; andere 1-basiert.
            % Wir setzen hier 1..n (oft ok). Falls dein Robot nicht reagiert:
            % -> auf (i-1) ändern.
            nJ = numel(dq_cmd);
            msg.JointSpeeds = repmat(msg.JointSpeeds, 1, nJ); %#ok<AGROW> (MATLAB passt ggf. an)

            for i = 1:nJ
                msg.JointSpeeds(i).JointIdentifier = uint32(i); % ggf. uint32(i-1)
                msg.JointSpeeds(i).Value          = double(dq_cmd(i));
                msg.JointSpeeds(i).Duration       = 0;
            end
            send(pub, msg);

        case "trajectory_msgs/JointTrajectory"
            % Fallback: JointTrajectory mit velocities
            msg = rosmessage(pub);
            msg.JointNames = cellstr(cfg.jointNames);

            pt = rosmessage("trajectory_msgs/JointTrajectoryPoint");
            pt.Velocities = double(dq_cmd).';
            pt.TimeFromStart = rosduration(1/cfg.rateHz);

            msg.Points = pt;
            send(pub, msg);

        otherwise
            error("Unsupported msg type: %s", msgType);
    end
end

function [q, dq, tstamp] = waitForJointState(stateSub, jointNames, timeout_s)
% Wartet auf joint_state, mappt nach jointNames Reihenfolge
    msg = receive(stateSub, timeout_s);
    if isempty(msg)
        error("Kein joint_state innerhalb timeout empfangen.");
    end

    % Zeitstempel (wenn Header da)
    try
        tstamp = double(msg.Header.Stamp.Sec) + 1e-9*double(msg.Header.Stamp.Nsec);
    catch
        tstamp = now*86400; % fallback: seconds since midnight (grob)
    end

    names = string(msg.Name);
    nJ = numel(jointNames);
    q  = zeros(nJ,1);
    dq = zeros(nJ,1);

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

function y = saturate(x, lim)
    y = max(min(x, lim), -lim);
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

% Was du typischerweise anpassen musst (2 Minuten)
% 
% cfg.ns (Namespace) und cfg.cmdTopic (Velocity Topic)
% 
% oft ist es /my_gen3/in/joint_speeds oder ähnlich.
% 
% cfg.jointNames passend zu deinem /joint_states (Namen müssen exakt matchen).
% 
% Falls der Roboter nicht reagiert, ist es sehr oft eins von:
% 
% falsches Topic,
% 
% falscher Message-Typ,
% 
% falsche JointIdentifier-Basis (0-basiert statt 1-basiert) → in sendJointVelocity bei JointIdentifier auf i-1 ändern.
% 
% Wenn du mir sagst, welchen Kinova (Gen3 / Gen3 Lite / Jaco) und welcher Treiber (kortex_driver, ros2_kortex, MATLAB Support Package), kann ich dir das Script auch so "hart verdrahten", dass es ohne Topic-Raten direkt läuft.
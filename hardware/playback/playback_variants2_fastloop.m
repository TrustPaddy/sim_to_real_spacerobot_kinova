%% playback_variants.m
% Faehrt mehrere dq_cmd-Playback-Varianten auf dem Kinova Gen3 durch.
% Pro Variante:
%   - Fahrt zur definierten Startpose (aus startpose_library)
%   - Zeit-Skalierung der Trajektorie (gleiche Bahn, andere Dauer)
%   - Velocity-Playback + Feedback-Logging
%   - Speicherung via run_logger
%
% Abhaengigkeiten (muessen im Pfad liegen):
%   - run_logger.m
%   - startpose_library.m
%   - kortexApiMexInterface (Kinova-MEX)
%   - URDF-Datei
%   - dq_cmd.mat
%
% SICHERHEIT:
%   - E-STOP in Reichweite.
%   - CTRL+C loest Zero-Velocity + Disconnect aus.
%   - Zwischen allen Laeufen faehrt der Arm zuerst zur Safe-Pose.

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg.robotIP        = '192.168.0.10';
cfg.user           = 'admin';
cfg.password       = 'admin';
cfg.sessionTimeout = uint32(60000);
cfg.controlTimeout = uint32(2000);
cfg.nJoints        = 7;

% Trajektoriendatei (Original bei 40 Hz erwartet)
cfg.trajFile  = '../dq_cmd.mat';
cfg.dqVarName = 'dq';
cfg.tVarName  = 't';
cfg.dqUnit    = 'rad/s';

% Harte Zeitgrenze pro Lauf (nach Zeit-Skalierung). inf = volle Datei.
cfg.playDuration_s = inf;

% Sicherheitslimits
cfg.dqMax_degPerSec      = 60.0;
cfg.speedCmdDuration     = 0;
cfg.maxTimingOverrun_s   = 0.200;
cfg.abortOnFeedbackError = true;

% Playback-Performance
% Ziel: Der 40-Hz-Command-Loop soll moeglichst nur Kommandos senden.
% Feedback und FK sind teuer und koennen die Rate auf ca. 20 Hz begrenzen.
cfg.feedbackEvery         = 2;      % 1 = jedes Sample, 2 = jedes 2. Sample, inf = nie
cfg.computeFKOnline       = false;  % false = schneller; EE bleibt im Log NaN
cfg.printTimingEvery      = 20;     % alle N Samples Timing ausgeben; 0 = aus

% Startpose-Fahrt
cfg.startConstraint    = int32(0);
cfg.startSpeed         = 0;
cfg.startDuration      = 0;
cfg.waitAfterPosMove_s = 2.0;
cfg.motionTimeout_s    = 60;
cfg.positionTol_deg    = 1.0;

% URDF fuer Online-FK
cfg.urdfFile   = '../GEN3-7DOF-VISION_ARM_URDF_V12.urdf';
cfg.eeBodyName = 'end_effector_link';
cfg.toolOffset = [0; 0; 0];

% Safe-Pose fuer Zwischenstopps (Name aus startpose_library)
cfg.safePose = 'elbow_bent';

% run_logger Zielordner
cfg.runDir = 'runs';

%% =========================
%  EXPERIMENT-MATRIX
%  =========================
% Spalten: label, startpose-Name, timeScale, comment
%   timeScale = 1.0 : Original-Geschwindigkeit
%   timeScale = 2.0 : doppelt so lange, halbe Geschwindigkeit (gleiche Bahn)
%   timeScale = 0.5 : halb so lange, doppelte Geschwindigkeit (gleiche Bahn)

experiments = { ...
    'kin_baseline_training', 'training', 1.0,  'Baseline, Trainingspose, volle Geschwindigkeit' ; ...
    'kin_slow_training',     'training', 2.0,  'Halbe Geschwindigkeit, gleiche Bahn' ; ...
    'kin_fast_training',     'training', 0.75, 'Etwas schneller (33% mehr), gleiche Bahn' ; ...
};

%% =========================
%  STARTPOSEN LADEN UND VALIDIEREN
%  =========================
poses = startpose_library();
for i = 1:size(experiments, 1)
    pName = experiments{i, 2};
    assert(isfield(poses, pName), ...
        'Startpose "%s" nicht in startpose_library (Experiment %d).', pName, i);
end
assert(isfield(poses, cfg.safePose), ...
    'Safe-Pose "%s" nicht in startpose_library.', cfg.safePose);

%% =========================
%  TRAJEKTORIE LADEN
%  =========================
fprintf('== Lade Trajektorie aus %s ==\n', cfg.trajFile);
S = load(cfg.trajFile);
assert(isfield(S, cfg.dqVarName), 'Variable "%s" fehlt.', cfg.dqVarName);
assert(isfield(S, cfg.tVarName),  'Variable "%s" fehlt.', cfg.tVarName);

dq_raw = S.(cfg.dqVarName);
t_raw  = S.(cfg.tVarName);
t_raw  = t_raw(:);

if size(dq_raw, 2) ~= cfg.nJoints && size(dq_raw, 1) == cfg.nJoints
    dq_raw = dq_raw.';
end
assert(size(dq_raw, 2) == cfg.nJoints, 'dq muss Nx%d sein.', cfg.nJoints);
assert(size(dq_raw, 1) == numel(t_raw), 'Laenge von t und dq passen nicht.');
t_raw = t_raw - t_raw(1);

% Einheiten -> rad/s (ausgangsbasis fuer Skalierung)
switch lower(cfg.dqUnit)
    case 'rad/s'
        dq_raw_rad = dq_raw;
    case 'deg/s'
        dq_raw_rad = deg2rad(dq_raw);
    otherwise
        error('Unbekannte Einheit: %s', cfg.dqUnit);
end

rateHz_original = round(1 / median(diff(t_raw)));
fprintf('  Samples original : %d\n', numel(t_raw));
fprintf('  Dauer original   : %.3f s\n', t_raw(end));
fprintf('  Rate original    : %d Hz\n', rateHz_original);

%% =========================
%  URDF LADEN
%  =========================
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
fprintf('URDF: %s (EE: %s)\n', cfg.urdfFile, cfg.eeBodyName);

%% =========================
%  KINOVA VERBINDEN
%  =========================
fprintf('\n== Verbinde mit Kinova Gen3 @ %s ==\n', cfg.robotIP);
[errCode, apiHandle, ~] = kortexApiMexInterface( ...
    'CreateRobotApisWrapper', ...
    cfg.robotIP, cfg.user, cfg.password, ...
    cfg.sessionTimeout, cfg.controlTimeout);
if errCode ~= 0
    error('Verbindung fehlgeschlagen (errorCode=%d).', errCode);
end
fprintf('Verbunden (apiHandle=%d)\n', apiHandle);

cleanupObj = onCleanup(@() safeShutdown(apiHandle, cfg.nJoints)); %#ok<NASGU>

%% =========================
%  INITIAL-FAHRT ZUR SAFE-POSE
%  =========================
fprintf('\n== Initial-Fahrt zur Safe-Pose "%s" ==\n', cfg.safePose);
input('ENTER zum Start (CTRL+C abbrechen)... ', 's');
moveToPose(apiHandle, poses.(cfg.safePose), cfg);

%% =========================
%  HAUPTLOOP UEBER EXPERIMENTE
%  =========================
nExperiments = size(experiments, 1);
fprintf('\n== %d Experimente geplant ==\n', nExperiments);

successCount = 0;
for i = 1:nExperiments
    expLabel     = experiments{i, 1};
    expPoseName  = experiments{i, 2};
    expTimeScale = experiments{i, 3};
    expComment   = experiments{i, 4};
    expStartPose = poses.(expPoseName);

    fprintf('\n=========================================\n');
    fprintf(' Experiment %d/%d: %s\n', i, nExperiments, expLabel);
    fprintf('   startpose : %s = [%s]\n', expPoseName, ...
            strjoin(compose('%.1f', expStartPose), ' '));
    fprintf('   timeScale : %.2f\n', expTimeScale);
    fprintf('   comment   : %s\n', expComment);
    fprintf('=========================================\n');

    % --- Zeit-Skalierung anwenden ---
    % Gleiche Bahn: Zeit * timeScale, dq / timeScale
    t_scaled      = t_raw * expTimeScale;
    dq_scaled_rad = dq_raw_rad / expTimeScale;
    dq_scaled_deg = rad2deg(dq_scaled_rad);
    rateHz_scaled = round(1 / median(diff(t_scaled)));

    fprintf('   -> %d Samples, %.2f s, %d Hz, max |dq| = %.1f deg/s\n', ...
        numel(t_scaled), t_scaled(end), rateHz_scaled, ...
        max(abs(dq_scaled_deg), [], 'all'));

    % Ggf. Dauer begrenzen
    if isfinite(cfg.playDuration_s)
        idx = t_scaled <= cfg.playDuration_s + 1e-12;
        t_scaled      = t_scaled(idx);
        dq_scaled_deg = dq_scaled_deg(idx, :);
    end

    % Saettigung
    if max(abs(dq_scaled_deg), [], 'all') > cfg.dqMax_degPerSec
        warning('Saettigung aktiv: %.1f > %.1f deg/s. dq wird geklippt!', ...
                max(abs(dq_scaled_deg), [], 'all'), cfg.dqMax_degPerSec);
    end
    dq_scaled_deg = max(min(dq_scaled_deg, cfg.dqMax_degPerSec), -cfg.dqMax_degPerSec);

    % --- Fahrt zur Safe-Pose, dann zur Startpose ---
    fprintf('\n Fahrt zur Safe-Pose "%s"...\n', cfg.safePose);
    try
        moveToPose(apiHandle, poses.(cfg.safePose), cfg);
    catch ME
        warning('Safe-Pose-Fahrt fehlgeschlagen: %s. Ueberspringe.', ME.message);
        continue;
    end

    fprintf(' Fahrt zur Startpose "%s"...\n', expPoseName);
    try
        moveToPose(apiHandle, expStartPose, cfg);
    catch ME
        warning('Startpose-Fahrt fehlgeschlagen: %s. Ueberspringe.', ME.message);
        continue;
    end

    % --- ENTER zum Starten ---
    fprintf('\n Arbeitsraum frei? Arm in Startpose?\n');
    fprintf(' [ ] Kein Hindernis, kein Kabel, E-STOP in Reichweite\n');
    input(' ENTER zum Starten des Playbacks (CTRL+C abbrechen)... ', 's');

    % --- Playback ---
    log_run = [];
    try
        log_run = runPlayback(apiHandle, cfg, robot_rbt, ...
                              t_scaled, dq_scaled_deg, rateHz_scaled);
    catch ME
        warning('Playback fehlgeschlagen: %s', ME.message);
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
    end

    if isempty(log_run)
        fprintf(' Experiment "%s" uebersprungen (kein Log).\n', expLabel);
        continue;
    end

    % --- Log speichern ---
    data = struct( ...
        't',             log_run.t_send, ...
        't_loop',        log_run.t_loop, ...
        'q_measured',    log_run.q, ...
        'dq_cmd',        log_run.dq_cmd, ...
        'ee_measured',   log_run.ee, ...
        't_ref',         log_run.t_ref, ...
        'overrun',       log_run.overrun, ...
        'errCode',       log_run.errCode, ...
        'timing',        log_run.timing);

    meta = struct( ...
        'label',           expLabel, ...
        'source',          'real', ...
        'startpose',       expStartPose, ...
        'startposeName',   expPoseName, ...
        'timeScale',       expTimeScale, ...
        'dqCmdSource',     cfg.trajFile, ...
        'rateHz_original', rateHz_original, ...
        'rateHz_scaled',   rateHz_scaled, ...
        'rateHz_actual_median', log_run.rateHz_actual_median, ...
        'feedbackEvery',   cfg.feedbackEvery, ...
        'computeFKOnline', cfg.computeFKOnline, ...
        'playDurationSec', t_scaled(end), ...
        'dqMaxDegPerSec',  cfg.dqMax_degPerSec, ...
        'urdfFile',        cfg.urdfFile, ...
        'comment',         expComment);

    run_logger(data, meta, 'runDir', cfg.runDir);
    successCount = successCount + 1;
end

%% =========================
%  ENDFAHRT ZUR SAFE-POSE
%  =========================
fprintf('\n== %d/%d Experimente erfolgreich. Endfahrt zur Safe-Pose ==\n', ...
        successCount, nExperiments);
try
    moveToPose(apiHandle, poses.(cfg.safePose), cfg);
catch
    warning('Endfahrt zur Safe-Pose fehlgeschlagen.');
end

%% =========================
%  VERBINDUNG TRENNEN
%  =========================
fprintf('\n== Verbindung trennen ==\n');
errCode = kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
if errCode == 0
    fprintf('Verbindung getrennt.\n');
else
    warning('DestroyRobotApisWrapper meldete errorCode=%d.', errCode);
end
clear cleanupObj;
fprintf('Fertig.\n');


%% =========================
%  LOKALE FUNKTIONEN
%  =========================

function moveToPose(apiHandle, targetAngles, cfg)
    errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
        cfg.startConstraint, cfg.startSpeed, cfg.startDuration, targetAngles);
    if errCode ~= 0
        error('ReachJointAngles fehlgeschlagen (errorCode=%d).', errCode);
    end
    waitForMotionComplete(apiHandle, targetAngles, cfg);
    pause(cfg.waitAfterPosMove_s);

    [errCode, ~, fb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errCode ~= 0
        error('Feedback nach Pose-Fahrt fehlgeschlagen.');
    end
    q_now = extractJointPositions(fb, cfg.nJoints);
    maxErr = 0;
    for j = 1:cfg.nJoints
        maxErr = max(maxErr, abs(wrapAngleDeg(q_now(j) - targetAngles(j))));
    end
    if maxErr > cfg.positionTol_deg
        error('Pose nicht erreicht (maxErr = %.2f deg).', maxErr);
    end
    fprintf('   OK (maxErr = %.3f deg)\n', maxErr);
end

function log = runPlayback(apiHandle, cfg, robot_rbt, t_traj, dq_degps, rateHz)
    N = size(dq_degps, 1);
    log.t_send  = nan(N, 1);  % Zeitpunkt direkt nach SendJointSpeedCommand
    log.t_loop  = nan(N, 1);  % Zeitpunkt nach waitfor(r)
    log.t_ref   = t_traj;
    log.q       = nan(N, cfg.nJoints);
    log.dq_cmd  = nan(N, cfg.nJoints);
    log.ee      = nan(N, 3);
    log.errCode = nan(N, 1);
    log.overrun = false(N, 1);

    % Detailliertes Timing zur Diagnose des Flaschenhalses
    log.timing.tCmd   = nan(N, 1);
    log.timing.tFb    = nan(N, 1);
    log.timing.tFk    = nan(N, 1);
    log.timing.tCycle = nan(N, 1);

    if ~isfield(cfg, 'feedbackEvery') || isempty(cfg.feedbackEvery)
        cfg.feedbackEvery = 1;
    end
    if ~isfield(cfg, 'computeFKOnline') || isempty(cfg.computeFKOnline)
        cfg.computeFKOnline = true;
    end
    if ~isfield(cfg, 'printTimingEvery') || isempty(cfg.printTimingEvery)
        cfg.printTimingEvery = 0;
    end

    r = rateControl(rateHz);
    reset(r);

    fprintf(' Playback laeuft mit Sollrate %d Hz (dt = %.1f ms)...\n', ...
        rateHz, 1000 / rateHz);
    fprintf('   feedbackEvery   : %s\n', feedbackEveryToString(cfg.feedbackEvery));
    fprintf('   computeFKOnline : %d\n', cfg.computeFKOnline);

    tStart = tic;

    for k = 1:N
        cycleTic = tic;
        tNow = toc(tStart);
        if isfinite(cfg.playDuration_s) && tNow >= cfg.playDuration_s
            fprintf(' Harte Zeitgrenze erreicht bei Sample %d/%d.\n', k, N);
            break;
        end

        % -----------------------------------------------------------------
        % WICHTIG FUER 40 Hz:
        % Erst senden, dann optional Feedback/FK loggen.
        % Der Command-Pfad bleibt dadurch so kurz wie moeglich.
        % -----------------------------------------------------------------
        dq_k = dq_degps(k, :);

        cmdTic = tic;
        errCmd = kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, cfg.speedCmdDuration, dq_k, uint32(cfg.nJoints));
        log.timing.tCmd(k) = toc(cmdTic);

        log.t_send(k)    = toc(tStart);
        log.dq_cmd(k, :) = dq_k;
        log.errCode(k)   = errCmd;

        if errCmd ~= 0
            warning('SendJointSpeedCommand fehlgeschlagen (errorCode=%d).', errCmd);
            sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
            error('Velocity-Kommando fehlgeschlagen.');
        end

        % Feedback ist absichtlich optional/ausgeduennt, weil RefreshFeedback
        % oft der Hauptgrund ist, warum MATLAB nur ca. 20 Hz erreicht.
        doFeedback = isfinite(cfg.feedbackEvery) && cfg.feedbackEvery >= 1 && ...
                     (mod(k - 1, cfg.feedbackEvery) == 0);

        if doFeedback
            fbTic = tic;
            [errFb, ~, fb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
            log.timing.tFb(k) = toc(fbTic);

            if errFb ~= 0
                warning('Feedback-Fehler (errorCode=%d) bei Sample %d.', errFb, k);
                if cfg.abortOnFeedbackError
                    sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
                    error('Feedback verloren.');
                end
            else
                q_now = extractJointPositions(fb, cfg.nJoints);
                log.q(k, :) = q_now;

                if cfg.computeFKOnline
                    fkTic = tic;
                    log.ee(k, :) = forwardKinEE(robot_rbt, q_now, ...
                        cfg.eeBodyName, cfg.toolOffset);
                    log.timing.tFk(k) = toc(fkTic);
                end
            end
        end

        expectedTime = (k - 1) / rateHz;
        if log.t_send(k) - expectedTime > cfg.maxTimingOverrun_s
            log.overrun(k) = true;
        end

        waitfor(r);
        log.t_loop(k) = toc(tStart);
        log.timing.tCycle(k) = toc(cycleTic);

        if cfg.printTimingEvery > 0 && mod(k, cfg.printTimingEvery) == 0
            validSend = log.t_send(1:k);
            validSend = validSend(isfinite(validSend));
            if numel(validSend) >= 2
                effHz = 1 / median(diff(validSend));
            else
                effHz = nan;
            end

            fprintf(['   k=%4d/%4d | eff %.1f Hz | Cmd %.1f ms | Fb %.1f ms | ', ...
                     'FK %.1f ms | Cycle %.1f ms\n'], ...
                k, N, effHz, ...
                1000 * median(log.timing.tCmd(1:k),   'omitnan'), ...
                1000 * median(log.timing.tFb(1:k),    'omitnan'), ...
                1000 * median(log.timing.tFk(1:k),    'omitnan'), ...
                1000 * median(log.timing.tCycle(1:k), 'omitnan'));
        end
    end

    validSend = log.t_send(isfinite(log.t_send));
    if numel(validSend) >= 2
        log.rateHz_actual_median = 1 / median(diff(validSend));
        log.dt_actual_median_s   = median(diff(validSend));
    else
        log.rateHz_actual_median = nan;
        log.dt_actual_median_s   = nan;
    end

    nSent = sum(isfinite(log.t_send));
    fprintf(' Playback beendet: %d/%d Kommandos gesendet.\n', nSent, N);
    fprintf('   Effektive Command-Rate median: %.2f Hz (dt = %.1f ms)\n', ...
        log.rateHz_actual_median, 1000 * log.dt_actual_median_s);
    fprintf(' Zero-Velocity...\n');

    for kstop = 1:10
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
        pause(0.05);
    end

    % Falls FK online deaktiviert war, wird sie nach dem sicheren Stop fuer
    % die vorhandenen Feedback-Samples nachgerechnet. Das beeinflusst die
    % Playback-Rate nicht.
    if ~cfg.computeFKOnline
        idxFk = find(all(isfinite(log.q), 2));
        for ii = idxFk(:).'
            log.ee(ii, :) = forwardKinEE(robot_rbt, log.q(ii, :), ...
                cfg.eeBodyName, cfg.toolOffset);
        end
    end
end

function safeShutdown(apiHandle, nJ)
    try
        fprintf('\n[safeShutdown] Zero-Velocity...\n');
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, 0, zeros(1, nJ), uint32(nJ));
        pause(0.05);
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, 0, zeros(1, nJ), uint32(nJ));
    catch
    end
    try
        kortexApiMexInterface('StopAction', apiHandle);
    catch
    end
    try
        fprintf('[safeShutdown] Disconnect...\n');
        kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
    catch
    end
end

function sendZeroVelocity(apiHandle, nJ, duration)
    try
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, duration, zeros(1, nJ), uint32(nJ));
    catch
    end
end

function waitForMotionComplete(apiHandle, target, cfg)
    tStart = tic;
    stable = 0;
    while toc(tStart) < cfg.motionTimeout_s
        [errCode, ~, fb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
        if errCode == 0
            q = extractJointPositions(fb, cfg.nJoints);
            err = zeros(1, cfg.nJoints);
            for j = 1:cfg.nJoints
                err(j) = wrapAngleDeg(q(j) - target(j));
            end
            if all(abs(err) < cfg.positionTol_deg)
                stable = stable + 1;
                if stable >= 5
                    return;
                end
            else
                stable = 0;
            end
        end
        pause(0.1);
    end
    warning('Timeout beim Warten auf Bewegungsende.');
end

function q = extractJointPositions(fb, nJ)
    q = zeros(1, nJ);
    if numel(fb) == 1 && isfield(fb, 'position') && numel(fb.position) >= nJ
        q = double(fb.position(1:nJ));
        return;
    end
    for i = 1:min(nJ, numel(fb))
        if isfield(fb, 'position')
            q(i) = double(fb(i).position);
        end
    end
end

function p = forwardKinEE(robot_rbt, q_deg, eeBodyName, toolOffset)
    T = getTransform(robot_rbt, deg2rad(q_deg(:).'), char(eeBodyName));
    p_flange = T(1:3, 4);
    if nargin >= 4 && ~isempty(toolOffset) && any(toolOffset ~= 0)
        p = (p_flange + T(1:3, 1:3) * toolOffset(:)).';
    else
        p = p_flange.';
    end
end

function s = feedbackEveryToString(feedbackEvery)
    if ~isfinite(feedbackEvery)
        s = 'nie';
    else
        s = sprintf('jedes %d. Sample', round(feedbackEvery));
    end
end

function w = wrapAngleDeg(a)
    w = mod(a + 180, 360) - 180;
end
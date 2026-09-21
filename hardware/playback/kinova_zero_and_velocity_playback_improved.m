%% kinova_zero_and_velocity_playback_improved.m
% Kinova Gen3: Fahrt zur Startposition [0 0 0 0 0 0 0] + Abspielen der in
% dq_cmd.mat gespeicherten Gelenkgeschwindigkeits-Trajektorie.
%
% Verbesserungen gegenueber der Ursprungsversion:
%   1. Harte Playback-Dauerbegrenzung, z.B. cfg.playDuration_s = 8.5
%   2. Zusaetzlicher Zeitabbruch im Loop via toc(tStart)
%   3. Sauberer Zeitcheck: t(end), N, dt, N/rateHz, effektive Laufzeit
%   4. Logging von Send-Zeitpunkt und Loop-Endzeit
%   5. Optionale Geschwindigkeits-Skalierung
%   6. Robuster Umgang mit t_traj, falls t nicht bei 0 beginnt
%   7. Zero-Velocity-Phase separat geloggt/ausgegeben
%
% ERWARTETER INHALT dq_cmd.mat:
%   - dq : Nx7 Matrix, Gelenkgeschwindigkeiten in rad/s
%   - t  : Nx1 Vektor, Zeitstempel in Sekunden
%
% SICHERHEITSHINWEIS:
%   - [0 0 0 0 0 0 0] = Arm vertikal nach oben gestreckt!
%   - Arbeitsraum oben UND im Bewegungsbereich freiraeumen.
%   - E-STOP in Reichweite.
%   - CTRL+C loest Zero-Velocity + Disconnect aus.

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

% Trajektoriendatei
cfg.trajFile   = sk_path('data', 'dq_cmd', 'dq_cmd.mat');
cfg.dqVarName  = 'dq';
cfg.tVarName   = 't';
cfg.dqUnit     = 'rad/s';

% Harte Playback-Begrenzung
% Setze auf inf, wenn die komplette dq_cmd.mat abgespielt werden soll.
cfg.playDuration_s = 8.5;

% Optionaler Skalierungsfaktor fuer alle Gelenkgeschwindigkeiten.
% 1.0 = exakt gespeicherte Trajektorie.
% 0.5 = halb so schnell/kleinere Geschwindigkeit, aber bei gleicher Abspielzeit.
cfg.speedScale = 1.0;

% Startposition
cfg.startAngles     = [0 0 0 0 0 0 0];
cfg.startConstraint = int32(0);
cfg.startSpeed      = 0;
cfg.startDuration   = 0;

% Warten / Verifikation nach Positionsfahrt
cfg.waitAfterPosMove_s = 2.0;
cfg.motionTimeout_s    = 60;
cfg.positionTol_deg    = 1.0;

% Sicherheitslimits fuer Velocity-Playback
cfg.dqMax_degPerSec  = 25.0;
cfg.speedCmdDuration = 0;      % 0 = Geschwindigkeit gilt bis zum naechsten Kommando

% Zusatz-Sicherheitsabbruch
cfg.abortOnFeedbackError = true;
cfg.maxTimingOverrun_s   = 0.200;   % Warnung, falls Loop stark hinterherhaengt

% Referenztrajektorie fuer Plot
cfg.r      = 0.2;
cfg.center = [0.008, -0.017, 1.312 - 0.2];
cfg.yConst = 0;

% URDF fuer Forward-Kinematik
cfg.urdfFile   = sk_path("robot", "GEN3-7DOF-VISION_ARM_URDF_V12.urdf");
cfg.eeBodyName = "end_effector_link";

% Tool-Offset im EE-lokalen Frame [m]
cfg.toolOffset = [0; 0; 0.115];

% Logging
cfg.saveLog = true;
cfg.logFile = sk_path('data', 'hardware', 'kinova_velocity_playback_log.mat');

%% =========================
%  TRAJEKTORIE LADEN
%  =========================
fprintf('== Lade Trajektorie aus %s ==\n', cfg.trajFile);

S = load(cfg.trajFile);
assert(isfield(S, cfg.dqVarName), 'Variable "%s" nicht in %s gefunden.', cfg.dqVarName, cfg.trajFile);
assert(isfield(S, cfg.tVarName),  'Variable "%s" nicht in %s gefunden.', cfg.tVarName,  cfg.trajFile);

dq_raw = S.(cfg.dqVarName);
t_traj = S.(cfg.tVarName);
t_traj = t_traj(:);

% Dimensionen pruefen
if size(dq_raw, 2) ~= cfg.nJoints && size(dq_raw, 1) == cfg.nJoints
    dq_raw = dq_raw.';
end
assert(size(dq_raw, 2) == cfg.nJoints, ...
    'dq muss Nx%d sein, ist aber [%d x %d].', ...
    cfg.nJoints, size(dq_raw,1), size(dq_raw,2));
assert(size(dq_raw, 1) == numel(t_traj), ...
    'Laenge von t (%d) passt nicht zu dq (%d).', numel(t_traj), size(dq_raw,1));

% Zeit bei 0 starten lassen
t_original_start = t_traj(1);
t_traj = t_traj - t_original_start;

% Plausibilitaet der Zeitbasis
dt_vec = diff(t_traj);
assert(all(dt_vec > 0), 't_traj muss streng monoton steigen.');
dt_traj = median(dt_vec);
dt_jitter = max(abs(dt_vec - dt_traj));

% Einheit -> deg/s
switch lower(cfg.dqUnit)
    case 'rad/s'
        dq_degps = rad2deg(dq_raw);
    case 'deg/s'
        dq_degps = dq_raw;
    otherwise
        error('Unbekannte Einheit: %s', cfg.dqUnit);
end

% Harte Begrenzung auf cfg.playDuration_s
if isfinite(cfg.playDuration_s)
    idx = t_traj <= cfg.playDuration_s + 1e-12;
    if ~any(idx)
        error('Nach Begrenzung auf %.3f s bleiben keine Samples uebrig.', cfg.playDuration_s);
    end
    dq_raw   = dq_raw(idx, :);
    dq_degps = dq_degps(idx, :);
    t_traj   = t_traj(idx);
end

% Nach Begrenzung neu berechnen
N = size(dq_degps, 1);
if N >= 2
    dt_traj = median(diff(t_traj));
    rateHz_exact = 1 / dt_traj;
    rateHz = round(rateHz_exact);
else
    error('Trajektorie hat zu wenige Samples.');
end

% Geschwindigkeit skalieren
dq_degps = cfg.speedScale * dq_degps;

fprintf('\n== Trajektorie geladen ==\n');
fprintf('  Original t_start         : %.6f s\n', t_original_start);
fprintf('  Samples nach Begrenzung  : %d\n', N);
fprintf('  t_traj(1)                : %.6f s\n', t_traj(1));
fprintf('  t_traj(end)              : %.6f s\n', t_traj(end));
fprintf('  dt median                : %.6f s\n', dt_traj);
fprintf('  dt jitter max            : %.6f s\n', dt_jitter);
fprintf('  rateHz exact             : %.3f Hz\n', rateHz_exact);
fprintf('  rateHz verwendet         : %d Hz\n', rateHz);
fprintf('  Dauer aus (N-1)*dt       : %.6f s\n', (N-1) * dt_traj);
fprintf('  Dauer aus N/rateHz       : %.6f s\n', N / rateHz);
fprintf('  Harte Playback-Dauer     : %.6f s\n', cfg.playDuration_s);
fprintf('  Speed Scale              : %.3f\n', cfg.speedScale);

fprintf('\n  Max |dq| pro Joint vor Saettigung [deg/s]:\n');
for j = 1:cfg.nJoints
    fprintf('    J%d: %6.2f\n', j, max(abs(dq_degps(:,j))));
end

% Saettigung
if any(abs(dq_degps) > cfg.dqMax_degPerSec, 'all')
    warning('Trajektorie ueberschreitet dqMax = %.1f deg/s. Werte werden geklippt!', cfg.dqMax_degPerSec);
end
dq_degps = max(min(dq_degps, cfg.dqMax_degPerSec), -cfg.dqMax_degPerSec);

fprintf('\n  Max |dq| nach Saettigung [deg/s]: %.3f\n', max(abs(dq_degps), [], 'all'));

if any(abs(dq_degps(end,:)) > 0.05)
    fprintf('  HINWEIS: Letzter Sample ist nicht Null (max |dq_end| = %.3f deg/s).\n', ...
            max(abs(dq_degps(end,:))));
    fprintf('           Script sendet am Ende automatisch Zero-Velocity.\n');
end

%% =========================
%  REFERENZTRAJEKTORIE FUER PLOT
%  =========================
cfg.maxDuration = t_traj(end);
cfg.Ts          = dt_traj;
cfg.omega       = pi / max(cfg.maxDuration, eps);

t_ref    = t_traj;
x_ref    = cfg.center(1) + cfg.r * sin(cfg.omega * t_ref);
y_ref    = cfg.center(2) + cfg.yConst * t_ref;
z_ref    = cfg.center(3) + cfg.r * cos(cfg.omega * t_ref);
traj_ref = [x_ref y_ref z_ref];

fprintf('\n  Referenz: Halbkreis, r=%.3f m, center=[%.3f %.3f %.3f], omega=%.3f rad/s\n', ...
    cfg.r, cfg.center(1), cfg.center(2), cfg.center(3), cfg.omega);

%% =========================
%  ROBOT-MODELL LADEN
%  =========================
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
fprintf('URDF geladen: %s (EE-Body: %s)\n', cfg.urdfFile, cfg.eeBodyName);

%% =========================
%  VERBINDUNG HERSTELLEN
%  =========================
fprintf('\n== Verbinde mit Kinova Gen3 @ %s ==\n', cfg.robotIP);

[errCode, apiHandle, ~] = kortexApiMexInterface( ...
    'CreateRobotApisWrapper', ...
    cfg.robotIP, cfg.user, cfg.password, ...
    cfg.sessionTimeout, cfg.controlTimeout);

if errCode ~= 0
    error('Verbindung fehlgeschlagen! errorCode=%d.', errCode);
end
fprintf('Verbunden (apiHandle=%d)\n', apiHandle);

cleanupObj = onCleanup(@() safeShutdown(apiHandle, cfg.nJoints)); %#ok<NASGU>

%% =========================
%  INITIALES FEEDBACK
%  =========================
[errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
if errCode ~= 0
    error('Sensor-Feedback fehlgeschlagen (errorCode=%d).', errCode);
end

q_current  = extractJointPositions(actuatorFb, cfg.nJoints);
ee_current = forwardKinEE(robot_rbt, q_current, cfg.eeBodyName, cfg.toolOffset);

fprintf('\nAktuelle EE-Position via FK + ToolOffset: [%.3f  %.3f  %.3f] m\n', ee_current);
fprintf('\nAktuelle Position vs. Startposition:\n');
fprintf('  Joint |   Ist [deg] |  Ziel [deg] |  Delta wrapped [deg]\n');
fprintf('  ------|-------------|-------------|----------------------\n');
for j = 1:cfg.nJoints
    delta = wrapAngleDeg(cfg.startAngles(j) - q_current(j));
    fprintf('    J%d  |   %7.2f   |   %6.2f    |   %+7.2f\n', ...
            j, q_current(j), cfg.startAngles(j), delta);
end

%% =========================
%  SCHRITT 1: FAHRT ZUR STARTPOSITION
%  =========================
fprintf('\n=========================================\n');
fprintf('  SCHRITT 1: Fahrt zur Startposition\n');
fprintf('  Ziel: [%s]\n', strjoin(string(cfg.startAngles), ' '));
fprintf('=========================================\n');
fprintf(' [ ] Arbeitsraum frei, auch oben\n');
fprintf(' [ ] Keine Kabel/Objekte im Bewegungsbereich\n');
fprintf(' [ ] E-STOP in Reichweite\n');
fprintf('=========================================\n');
input('ENTER zum Fortfahren (CTRL+C abbrechen)... ', 's');

errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
    cfg.startConstraint, cfg.startSpeed, cfg.startDuration, cfg.startAngles);
if errCode ~= 0
    error('ReachJointAngles fehlgeschlagen (errorCode=%d).', errCode);
end

fprintf('Kommando gesendet. Warte auf Bewegungsende...\n');
waitForMotionComplete(apiHandle, cfg.startAngles, cfg);
pause(cfg.waitAfterPosMove_s);

% Verifikation
[errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
if errCode ~= 0
    error('Feedback nach Startfahrt fehlgeschlagen (errorCode=%d).', errCode);
end

q_start = extractJointPositions(actuatorFb, cfg.nJoints);
fprintf('\nStartposition erreicht:\n');
maxErr = 0;
for j = 1:cfg.nJoints
    err = wrapAngleDeg(q_start(j) - cfg.startAngles(j));
    maxErr = max(maxErr, abs(err));
    fprintf('  J%d: %8.3f deg  (Abweichung: %+6.3f deg)\n', j, q_start(j), err);
end
fprintf('  Max. Abweichung: %.3f deg\n', maxErr);

if maxErr > cfg.positionTol_deg
    error('Startposition ausserhalb Toleranz. Playback wird aus Sicherheitsgruenden nicht gestartet.');
end

%% =========================
%  SCHRITT 2: SAFETY CHECK PLAYBACK
%  =========================
fprintf('\n=========================================\n');
fprintf('  SCHRITT 2: Velocity-Trajektorie abspielen\n');
fprintf('=========================================\n');
fprintf(' - Harte Dauergrenze     : %.3f s\n', cfg.playDuration_s);
fprintf(' - t_traj(end)           : %.3f s\n', t_traj(end));
fprintf(' - Sende-Rate            : %d Hz\n', rateHz);
fprintf(' - Samples               : %d\n', N);
fprintf(' - Erwartet N/rateHz     : %.3f s\n', N / rateHz);
fprintf(' - |dq|max alle          : %.2f deg/s\n', max(abs(dq_degps), [], 'all'));
fprintf(' - Saettigung            : %.1f deg/s\n', cfg.dqMax_degPerSec);
fprintf(' - Speed Scale           : %.2f\n', cfg.speedScale);
fprintf(' - CTRL+C: Zero-Velocity + Disconnect\n');
fprintf('=========================================\n');
input('ENTER zum Starten des Playbacks (CTRL+C abbrechen)... ', 's');

%% =========================
%  PLAYBACK
%  =========================
log.t_send   = nan(N, 1);       % Zeitpunkt direkt nach SendJointSpeedCommand
log.t_loop   = nan(N, 1);       % Zeitpunkt nach waitfor
log.t_ref    = t_traj;
log.q        = nan(N, cfg.nJoints);
log.dq_cmd   = nan(N, cfg.nJoints);
log.ee       = nan(N, 3);
log.errCode  = nan(N, 1);
log.overrun  = false(N, 1);

r = rateControl(rateHz);
reset(r);

fprintf('\nPlayback laeuft...\n');
tStart = tic;
kExecuted = 0;

for k = 1:N
    tNow = toc(tStart);

    % Harte Zeitgrenze: keine weiteren Geschwindigkeitskommandos nach cfg.playDuration_s
    if isfinite(cfg.playDuration_s) && tNow >= cfg.playDuration_s
        fprintf('Harte Playback-Dauer %.3f s erreicht. Stoppe bei Sample %d/%d.\n', ...
            cfg.playDuration_s, k, N);
        break;
    end

    % Feedback lesen
    [errFb, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errFb ~= 0
        warning('Feedback-Fehler (errorCode=%d) bei Sample %d.', errFb, k);
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
        if cfg.abortOnFeedbackError
            error('Feedback waehrend Playback verloren.');
        else
            continue;
        end
    end

    q_now  = extractJointPositions(actuatorFb, cfg.nJoints);
    ee_now = forwardKinEE(robot_rbt, q_now, cfg.eeBodyName, cfg.toolOffset);

    % Velocity-Kommando fuer diesen Sample
    dq_k = dq_degps(k, :);

    errCmd = kortexApiMexInterface('SendJointSpeedCommand', ...
        apiHandle, cfg.speedCmdDuration, dq_k, uint32(cfg.nJoints));
    if errCmd ~= 0
        warning('SendJointSpeedCommand fehlgeschlagen (errorCode=%d) bei Sample %d. Stoppe.', errCmd, k);
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
        error('Velocity-Kommando fehlgeschlagen.');
    end

    % Logging direkt nach Senden
    log.t_send(k)   = toc(tStart);
    log.q(k, :)     = q_now;
    log.dq_cmd(k,:) = dq_k;
    log.ee(k, :)    = ee_now;
    log.errCode(k)  = errCmd;

    % Timing-Warnung
    expectedTime = (k-1) / rateHz;
    if log.t_send(k) - expectedTime > cfg.maxTimingOverrun_s
        log.overrun(k) = true;
    end

    waitfor(r);

    log.t_loop(k) = toc(tStart);
    kExecuted = k;
end

playbackTimeBeforeZero = toc(tStart);

% Sofort Zero-Velocity senden
fprintf('Playback-Kommandos beendet nach %.3f s. Sende Zero-Velocity...\n', playbackTimeBeforeZero);
for kstop = 1:10
    sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
    pause(0.05);
end
zeroEndTime = toc(tStart);

fprintf('\n== Zeitbilanz ==\n');
fprintf('  Ausgefuehrte Samples          : %d/%d\n', kExecuted, N);
fprintf('  Zeit bis Stop-Kommando        : %.3f s\n', playbackTimeBeforeZero);
fprintf('  Zero-Velocity Zusatzphase     : %.3f s\n', zeroEndTime - playbackTimeBeforeZero);
fprintf('  Gesamt bis Zero-Ende          : %.3f s\n', zeroEndTime);
fprintf('  Letzter Send-Zeitpunkt        : %.3f s\n', lastFinite(log.t_send));
fprintf('  Letztes Loop-Ende             : %.3f s\n', lastFinite(log.t_loop));
fprintf('  Timing-Overrun Samples        : %d\n', nnz(log.overrun));

% Finales Feedback
[errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
if errCode == 0
    q_end = extractJointPositions(actuatorFb, cfg.nJoints);
    fprintf('\nEndposition nach Playback:\n');
    for j = 1:cfg.nJoints
        delta = wrapAngleDeg(q_end(j) - q_start(j));
        fprintf('  J%d: %8.3f deg  (Delta zur Startpose: %+7.3f deg)\n', ...
                j, q_end(j), delta);
    end
else
    warning('Finales Feedback fehlgeschlagen (errorCode=%d).', errCode);
end

%% =========================
%  LOG SPEICHERN + PLOT
%  =========================
log.traj_ref   = traj_ref;
log.t_refEE    = t_ref;
log.kExecuted  = kExecuted;
log.playbackTimeBeforeZero = playbackTimeBeforeZero;
log.zeroEndTime = zeroEndTime;

if cfg.saveLog
    save(cfg.logFile, 'cfg', 'log');
    fprintf('\nLog gespeichert: %s\n', cfg.logFile);
end

valid = ~isnan(log.t_send);

figure('Name', 'Kinova Velocity Playback');

subplot(2,1,1);
plot(log.t_send(valid), log.dq_cmd(valid,:));
grid on; xlabel('t_{send} [s]'); ylabel('dq\_cmd [deg/s]');
title('Kommandierte Gelenkgeschwindigkeiten');
legend(compose("J%d", 1:cfg.nJoints), 'Location', 'best');

subplot(2,1,2);
plot(log.t_send(valid), log.q(valid,:));
grid on; xlabel('t_{send} [s]'); ylabel('q [deg]');
title('Gemessene Gelenkwinkel');
legend(compose("J%d", 1:cfg.nJoints), 'Location', 'best');

figure('Name', 'Kinova EE Trajectory: Ist vs. Soll (xz)');
plot(log.ee(valid,1), log.ee(valid,3), 'b-', 'LineWidth', 1.5); hold on;
plot(traj_ref(:,1), traj_ref(:,3), 'r--', 'LineWidth', 1.5);

if any(valid)
    firstIdx = find(valid, 1, 'first');
    plot(log.ee(firstIdx,1), log.ee(firstIdx,3), 'go', 'MarkerFaceColor','g');
end
plot(traj_ref(1,1), traj_ref(1,3), 'ks', 'MarkerFaceColor','k');

grid on; axis equal;
xlabel('x [m]'); ylabel('z [m]');
legend('Ist via FK + ToolOffset', 'Soll-Referenz', 'Start Ist', 'Start Soll', ...
       'Location', 'best');
title('End-Effector Trajektorie in xz-Ebene: Ist vs. Soll');

figure('Name', 'Timing Playback');
plot(log.t_send(valid), log.t_send(valid) - log.t_ref(valid), 'LineWidth', 1.2);
grid on;
xlabel('t_{send} [s]');
ylabel('t_{send} - t_{ref} [s]');
title('Timing-Abweichung: gesendete Zeit minus Referenzzeit');

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


%% ========================================================================
%  LOKALE FUNKTIONEN
%  ========================================================================

function safeShutdown(apiHandle, nJ)
% Wird bei Fehler oder CTRL+C automatisch ausgefuehrt:
% sendet Null-Geschwindigkeit, stoppt Action und trennt die Verbindung.
    try
        fprintf('\n[safeShutdown] Sende Zero-Velocity...\n');
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
        fprintf('[safeShutdown] Trenne Verbindung...\n');
        kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
    catch
    end

    fprintf('[safeShutdown] Abgeschlossen.\n');
end

function sendZeroVelocity(apiHandle, nJ, duration)
    try
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, duration, zeros(1, nJ), uint32(nJ));
    catch
    end
end

function waitForMotionComplete(apiHandle, target, cfg)
% Wartet bis alle Gelenke innerhalb Toleranz liegen und 5x stabil bleiben.
    fprintf('   Warte auf Bewegungsende...');
    tStart = tic;
    stableCount = 0;

    while toc(tStart) < cfg.motionTimeout_s
        [errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);

        if errCode == 0
            q = extractJointPositions(actuatorFb, cfg.nJoints);
            err = zeros(1, cfg.nJoints);

            for j = 1:cfg.nJoints
                err(j) = wrapAngleDeg(q(j) - target(j));
            end

            if all(abs(err) < cfg.positionTol_deg)
                stableCount = stableCount + 1;
                if stableCount >= 5
                    fprintf(' OK (%.1fs)\n', toc(tStart));
                    return;
                end
            else
                stableCount = 0;
            end
        end

        pause(0.1);
    end

    warning('Timeout - Bewegung nicht innerhalb %.0fs abgeschlossen.', cfg.motionTimeout_s);
end

function q = extractJointPositions(actuatorFb, nJ)
% Extrahiert Gelenkwinkel [deg] aus dem Actuator-Feedback.
    q = zeros(1, nJ);

    if numel(actuatorFb) == 1 && isfield(actuatorFb, 'position') ...
            && numel(actuatorFb.position) >= nJ
        q = double(actuatorFb.position(1:nJ));
        return;
    end

    for i = 1:min(nJ, numel(actuatorFb))
        if isfield(actuatorFb, 'position')
            q(i) = double(actuatorFb(i).position);
        end
    end
end

function p = forwardKinEE(robot_rbt, q_deg, eeBodyName, toolOffset)
% Berechnet die End-Effector-Position [x y z] in Metern aus Gelenkwinkeln [deg].
% Optionaler toolOffset wird im EE-lokalen Frame interpretiert.
    T = getTransform(robot_rbt, deg2rad(q_deg(:).'), char(eeBodyName));
    p_flange = T(1:3, 4);

    if nargin >= 4 && ~isempty(toolOffset) && any(toolOffset ~= 0)
        p_tool = p_flange + T(1:3, 1:3) * toolOffset(:);
    else
        p_tool = p_flange;
    end

    p = p_tool.';
end

function wrapped = wrapAngleDeg(angle)
    wrapped = mod(angle + 180, 360) - 180;
end

function y = lastFinite(x)
    idx = find(isfinite(x), 1, 'last');
    if isempty(idx)
        y = NaN;
    else
        y = x(idx);
    end
end

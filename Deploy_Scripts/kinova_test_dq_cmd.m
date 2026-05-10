%% kinova_zero_and_velocity_playback.m
% Kinova Gen3: Fahrt zur Startposition [0 0 0 0 0 0 0] + Abspielen der in
% dq_cmd.mat gespeicherten Gelenkgeschwindigkeits-Trajektorie.
%
% Ablauf:
%   1. Lade Trajektorie (dq, t) aus dq_cmd.mat
%   2. Verbindung via kortexApiMexInterface (MEX-Interface, wie in kinova_test.m)
%   3. Fahrt zur Startposition [0 0 0 0 0 0 0] mit ReachJointAngles
%   4. Verifikation der Startposition
%   5. Abspielen der Velocity-Trajektorie mit SendJointSpeedCommand bei 40 Hz
%   6. Explizites Zero-Velocity am Ende (Trajektorie endet NICHT bei 0!)
%   7. Safe shutdown bei Fehler/CTRL+C via onCleanup
%   8. Logging + Plot
%
% ERWARTETER INHALT dq_cmd.mat:
%   - dq : Nx7 Matrix, Gelenkgeschwindigkeiten in rad/s
%   - t  : Nx1 Vektor, Zeitstempel in Sekunden (aequidistant)
%
% SICHERHEITSHINWEIS:
%   - [0 0 0 0 0 0 0] = Arm vertikal nach oben gestreckt!
%     => Deckenfreiheit pruefen.
%   - Danach bewegt die Trajektorie den Arm ab dieser Pose weiter (J2, J4, J6).
%   - Arbeitsraum oben UND im Bewegungsbereich freiraeumen.
%   - E-STOP in Reichweite. CTRL+C loest Zero-Velocity + Disconnect aus.

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg.robotIP        = '192.168.0.10';      % <-- ggf. anpassen
cfg.user           = 'admin';
cfg.password       = 'admin';
cfg.sessionTimeout = uint32(60000);
cfg.controlTimeout = uint32(2000);
cfg.nJoints        = 7;

% Trajektoriendatei
cfg.trajFile   = 'dq_cmd.mat';
cfg.dqVarName  = 'dq';
cfg.tVarName   = 't';
cfg.dqUnit     = 'rad/s';                 % dq_cmd.mat ist in rad/s gespeichert

% Startposition
cfg.startAngles     = [0 0 0 0 0 0 0];
cfg.startConstraint = int32(0);           % 0 = Default-Speed (wie in test.m)
cfg.startSpeed      = 0;
cfg.startDuration   = 0;

% Warten / Verifikation nach Positionsfahrt
cfg.waitAfterPosMove_s = 2.0;
cfg.motionTimeout_s    = 60;
cfg.positionTol_deg    = 1.0;

% Sicherheitslimits fuer Velocity-Playback
cfg.dqMax_degPerSec  = 25.0;              % Saettigung [deg/s] (Daten-Max ~21.8)
cfg.speedCmdDuration = 0;                 % Duration-Param fuer SendJointSpeedCommand

% Referenztrajektorie (Soll-EE-Pfad: Halbkreis in xz-Ebene, URDF/Training-Frame)
cfg.r      = 0.2;
cfg.center = [0.008, -0.017, 1.312 - 0.2];   % [m], URDF/Training-Frame
cfg.yConst = 0;
% cfg.omega und cfg.Ts werden unten aus der Trajektoriendauer bestimmt.

% URDF fuer Forward-Kinematik (Ist-EE-Position aus q berechnen)
cfg.urdfFile   = "GEN3-7DOF-VISION_ARM_URDF_V12.urdf";
cfg.eeBodyName = "end_effector_link";

% Tool-Offset im EE-lokalen Frame [m].
% Die Kortex-Web-UI zeigt die "Tool Pose" INKLUSIVE Tool-Konfiguration
% (Greifer/Interface Module), die in der URDF nicht enthalten ist.
% Bei Gen3 + Standard-Gripper ist der Tool-Tip ~0.115 m ueber end_effector_link
% entlang der EE-lokalen +Z-Achse. Auf 0 setzen fuer Flange-Pose.
cfg.toolOffset = [0; 0; 0.115];

% Logging
cfg.saveLog = true;
cfg.logFile = 'kinova_velocity_playback_log.mat';

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

% Dimensionen pruefen (Nx7 erwartet; 7xN automatisch transponieren)
if size(dq_raw, 2) ~= cfg.nJoints && size(dq_raw, 1) == cfg.nJoints
    dq_raw = dq_raw.';
end
assert(size(dq_raw, 2) == cfg.nJoints, ...
    'dq muss Nx%d sein, ist aber [%d x %d].', ...
    cfg.nJoints, size(dq_raw,1), size(dq_raw,2));
assert(size(dq_raw, 1) == numel(t_traj), ...
    'Laenge von t (%d) passt nicht zu dq (%d).', numel(t_traj), size(dq_raw,1));

% Einheit -> deg/s (Kinova SendJointSpeedCommand erwartet deg/s)
switch lower(cfg.dqUnit)
    case 'rad/s', dq_degps = rad2deg(dq_raw);
    case 'deg/s', dq_degps = dq_raw;
    otherwise, error('Unbekannte Einheit: %s', cfg.dqUnit);
end

% Abtastrate aus t
dt_traj = median(diff(t_traj));
rateHz  = round(1 / dt_traj);

fprintf('  Samples : %d\n', size(dq_degps, 1));
fprintf('  Dauer   : %.3f s\n', t_traj(end) - t_traj(1));
fprintf('  dt      : %.4f s  (%.1f Hz)\n', dt_traj, rateHz);
fprintf('  Max |dq| pro Joint [deg/s]:\n');
for j = 1:cfg.nJoints
    fprintf('    J%d: %6.2f\n', j, max(abs(dq_degps(:,j))));
end

% Saettigung
if any(abs(dq_degps) > cfg.dqMax_degPerSec, 'all')
    warning('Trajektorie ueberschreitet dqMax = %.1f deg/s. Werte werden geklippt!', cfg.dqMax_degPerSec);
end
dq_degps = max(min(dq_degps, cfg.dqMax_degPerSec), -cfg.dqMax_degPerSec);

% Hinweis falls letzter Sample nicht 0
if any(abs(dq_degps(end,:)) > 0.05)
    fprintf('  HINWEIS: Letzter Sample ist nicht Null (max |dq_end| = %.3f deg/s).\n', ...
            max(abs(dq_degps(end,:))));
    fprintf('           Script sendet am Ende automatisch Zero-Velocity.\n');
end

%% =========================
%  REFERENZTRAJEKTORIE (SOLL-EE-PFAD)
%  =========================
cfg.maxDuration = t_traj(end) - t_traj(1);
cfg.Ts          = dt_traj;
cfg.omega       = pi / cfg.maxDuration;

t_ref    = (0:cfg.Ts:cfg.maxDuration).';
x_ref    = cfg.center(1) + cfg.r * sin(cfg.omega * t_ref);
y_ref    = cfg.center(2) + cfg.yConst * t_ref;
z_ref    = cfg.center(3) + cfg.r * cos(cfg.omega * t_ref);
traj_ref = [x_ref y_ref z_ref];

fprintf('  Referenz  : Halbkreis, r=%.3f m, center=[%.3f %.3f %.3f], omega=%.3f rad/s\n', ...
    cfg.r, cfg.center(1), cfg.center(2), cfg.center(3), cfg.omega);

%% =========================
%  ROBOT-MODELL LADEN (fuer FK)
%  =========================
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
fprintf('URDF geladen: %s (EE-Body: %s)\n', cfg.urdfFile, cfg.eeBodyName);

%% =========================
%  VERBINDUNG HERSTELLEN (MEX)
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

% Safe shutdown bei Fehler / CTRL+C
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
fprintf('\nAktuelle EE-Position (via FK): [%.3f  %.3f  %.3f] m\n', ee_current);
fprintf('\nAktuelle Position vs. Startposition:\n');
fprintf('  Joint |   Ist [deg] |  Ziel [deg] |  Delta (wrapped) [deg]\n');
fprintf('  ------|-------------|-------------|-----------------------\n');
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
fprintf(' [ ] Arbeitsraum frei (auch OBEN, Arm steht senkrecht!)\n');
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
q_start = cfg.startAngles;
[errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
if errCode == 0
    q_start = extractJointPositions(actuatorFb, cfg.nJoints);
    fprintf('\nStartposition erreicht:\n');
    maxErr = 0;
    for j = 1:cfg.nJoints
        err = wrapAngleDeg(q_start(j) - cfg.startAngles(j));
        maxErr = max(maxErr, abs(err));
        fprintf('  J%d: %8.3f deg  (Abweichung: %+6.3f deg)\n', j, q_start(j), err);
    end
    fprintf('  Max. Abweichung: %.3f deg\n', maxErr);
end

%% =========================
%  SCHRITT 2: SAFETY CHECK PLAYBACK
%  =========================
fprintf('\n=========================================\n');
fprintf('  SCHRITT 2: Velocity-Trajektorie abspielen\n');
fprintf('=========================================\n');
fprintf(' - Dauer         : %.2f s\n', t_traj(end));
fprintf(' - Sende-Rate    : %d Hz\n', rateHz);
fprintf(' - Samples       : %d\n', size(dq_degps, 1));
fprintf(' - |dq|max (alle): %.2f deg/s\n', max(abs(dq_degps), [], 'all'));
fprintf(' - Saettigung    : %.1f deg/s\n', cfg.dqMax_degPerSec);
fprintf(' - CTRL+C loest Zero-Velocity + Disconnect aus.\n');
fprintf('=========================================\n');
input('ENTER zum Starten des Playbacks (CTRL+C abbrechen)... ', 's');

%% =========================
%  PLAYBACK
%  =========================
N = size(dq_degps, 1);

% Log-Buffer vorallokieren
log.t_cmd   = zeros(N, 1);
log.t_ref   = t_traj;
log.q       = zeros(N, cfg.nJoints);
log.dq_cmd  = zeros(N, cfg.nJoints);
log.ee      = nan(N, 3);                  % Ist-EE-Position [x y z] in m
log.errCode = zeros(N, 1);

r = rateControl(rateHz);
reset(r);

fprintf('Playback laeuft...\n');
tStart = tic;
for k = 1:N
    % Feedback lesen (Watchdog)
    [errFb, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errFb ~= 0
        warning('Feedback-Fehler (errorCode=%d) bei Sample %d. Stoppe sofort.', errFb, k);
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
        error('Feedback waehrend Playback verloren.');
    end
    q_now  = extractJointPositions(actuatorFb, cfg.nJoints);
    ee_now = forwardKinEE(robot_rbt, q_now, cfg.eeBodyName, cfg.toolOffset);

    % Velocity-Kommando fuer diesen Sample
    dq_k = 0.5 * dq_degps(k, :);

    errCmd = kortexApiMexInterface('SendJointSpeedCommand', ...
        apiHandle, cfg.speedCmdDuration, dq_k, uint32(cfg.nJoints));
    if errCmd ~= 0
        warning('SendJointSpeedCommand fehlgeschlagen (errorCode=%d) bei Sample %d. Stoppe.', errCmd, k);
        sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
        error('Velocity-Kommando fehlgeschlagen.');
    end

    % Logging
    log.t_cmd(k)     = toc(tStart);
    log.q(k, :)      = q_now;
    log.dq_cmd(k, :) = dq_k;
    log.ee(k, :)     = ee_now;
    log.errCode(k)   = errCmd;

    waitfor(r);
end

% Playback-Ende: explizit Zero-Velocity senden (Trajektorie endet nicht bei 0)
fprintf('Playback beendet nach %.3f s. Sende Zero-Velocity...\n', toc(tStart));
for kstop = 1:10
    sendZeroVelocity(apiHandle, cfg.nJoints, cfg.speedCmdDuration);
    pause(0.05);
end

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
end

%% =========================
%  LOG SPEICHERN + PLOT
%  =========================
log.traj_ref = traj_ref;
log.t_refEE  = t_ref;

if cfg.saveLog
    save(cfg.logFile, 'cfg', 'log');
    fprintf('\nLog gespeichert: %s\n', cfg.logFile);
end

figure('Name', 'Kinova Velocity Playback');

subplot(2,1,1);
plot(log.t_cmd, log.dq_cmd);
grid on; xlabel('t [s]'); ylabel('dq\_cmd [deg/s]');
title('Kommandierte Gelenkgeschwindigkeiten');
legend(compose("J%d", 1:cfg.nJoints), 'Location', 'best');

subplot(2,1,2);
plot(log.t_cmd, log.q);
grid on; xlabel('t [s]'); ylabel('q [deg]');
title('Gemessene Gelenkwinkel');
legend(compose("J%d", 1:cfg.nJoints), 'Location', 'best');

% ---- End-Effector: Ist vs. Soll (xz-Ebene) ----
figure('Name', 'Kinova EE Trajectory: Ist vs. Soll (xz)');

% 2D-Vergleich in der xz-Ebene (z auf der y-Achse)
plot(log.ee(:,1),   log.ee(:,3),   'b-',  'LineWidth', 1.5); hold on;
plot(traj_ref(:,1), traj_ref(:,3), 'r--', 'LineWidth', 1.5);
plot(log.ee(1,1),   log.ee(1,3),   'go', 'MarkerFaceColor','g'); % Start Ist
plot(traj_ref(1,1), traj_ref(1,3), 'ks', 'MarkerFaceColor','k'); % Start Soll
grid on; axis equal;
xlabel('x [m]'); ylabel('z [m]');
legend('Ist (gemessen)', 'Soll (Referenz)', 'Start Ist', 'Start Soll', ...
       'Location', 'best');
title('End-Effector Trajektorie in xz-Ebene: Ist vs. Soll');

%% =========================
%  VERBINDUNG TRENNEN
%  =========================
fprintf('\n== Verbindung trennen ==\n');
errCode = kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
if errCode == 0
    fprintf('Verbindung getrennt.\n');
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
    catch, end
    try
        kortexApiMexInterface('StopAction', apiHandle);
    catch, end
    try
        fprintf('[safeShutdown] Trenne Verbindung...\n');
        kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
    catch, end
    fprintf('[safeShutdown] Abgeschlossen.\n');
end

function sendZeroVelocity(apiHandle, nJ, duration)
    try
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, duration, zeros(1, nJ), uint32(nJ));
    catch, end
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
% Extrahiert Gelenkwinkel [deg] aus dem Actuator-Feedback (robust fuer beide Formate).
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
% Berechnet die End-Effector-Position [x y z] in Metern aus den
% Gelenkwinkeln (in Grad) via Forward-Kinematik (URDF-Basis-Frame).
% Optional: toolOffset [3x1] in EE-lokalen Koordinaten (rotiert mit dem EE mit).
    T = getTransform(robot_rbt, deg2rad(q_deg(:).'), char(eeBodyName));
    p_flange = T(1:3, 4);
    if nargin >= 4 && ~isempty(toolOffset) && any(toolOffset ~= 0)
        p_tool = p_flange + T(1:3, 1:3) * toolOffset(:);
    else
        p_tool = p_flange;
    end
    p = p_tool.';   % Zeilenvektor [x y z] in m
end

function wrapped = wrapAngleDeg(angle)
    wrapped = mod(angle + 180, 360) - 180;
end
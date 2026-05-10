%% kinova_velocity_test.m
% Sicherer Joint-Velocity-Jog-Test fuer Kinova Gen3 (7-DoF)
% Nutzt das MEX-Interface (kortexApiMexInterface) fuer echtes Velocity-Commanding.
%
% Funktionsweise:
%   - Verbindung via kortexApiMexInterface('CreateRobotApisWrapper', ...)
%   - Liest Sensor-Feedback mit 'RefreshFeedback'
%   - Sendet echte Velocity-Kommandos mit 'SendJointSpeedCommand'
%   - Safety: Saettigung, Watchdog, onCleanup -> stoppt sicher
%   - Logging: t, q_meas, dq_cmd
%
% SICHERHEITSHINWEIS:
%   - Arbeitsraum freiraeumen, E-STOP bereithalten!
%   - Script Abschnitt fuer Abschnitt ausfuehren.
%   - CTRL+C bricht sicher ab (onCleanup sendet Zero-Velocity + Disconnect).
%
% Voraussetzungen:
%   - Robotics System Toolbox
%   - Robotics System Toolbox Support Package for KINOVA Gen3 Manipulators
%   - Die MEX-Datei kortexApiMexInterface muss auf dem MATLAB-Pfad liegen

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg.robotIP        = '192.168.0.10';  % <-- IP deines Kinova Gen3 anpassen!
cfg.user           = 'admin';
cfg.password       = 'admin';
cfg.sessionTimeout = uint32(60000);   % Session-Timeout [ms]
cfg.controlTimeout = uint32(2000);    % Control-Timeout [ms]

cfg.nJoints        = 7;              % Gen3 7-DoF

% Sicherheitslimits & Testprofil
cfg.rateHz             = 100;        % Sende-/Log-Rate [Hz]
cfg.dqMax_degPerSec    = 15.0;       % Absolute Saettigung [deg/s]
cfg.testSpeed_degPerSec = 5.0;       % Testgeschwindigkeit [deg/s] – klein halten!
cfg.testDuration_s     = 1.5;        % Dauer pro Joint-Richtung [s]
cfg.restDuration_s     = 1.0;        % Stop-Phase zwischen Tests [s]
cfg.speedCmdDuration   = 0;          % Duration-Parameter fuer SendJointSpeedCommand
                                     % (0 = kein internes Timeout, wir steuern selbst)

% Test-Optionen
cfg.testBothDirections = true;       % true: testet +speed und -speed pro Joint
cfg.startFromRetract   = true;       % true: faehrt zuerst in Retract-Position
cfg.waitAfterRetract_s = 6.0;       % Wartezeit nach Retract-Kommando [s]

% Logging
cfg.saveLog  = true;
cfg.logFile  = 'kinova_velocity_test_log.mat';

%% =========================
%  VERBINDUNG HERSTELLEN (MEX-Interface)
%  =========================
fprintf('== Kinova Gen3 Verbindung (MEX-Interface) ==\n');

[errCode, apiHandle, ~] = kortexApiMexInterface( ...
    'CreateRobotApisWrapper', ...
    cfg.robotIP, cfg.user, cfg.password, ...
    cfg.sessionTimeout, cfg.controlTimeout);

if errCode ~= 0
    error('Verbindung fehlgeschlagen! errorCode=%d. IP/Netzwerk/Robot pruefen.', errCode);
end
fprintf('Verbunden mit Kinova Gen3 @ %s (apiHandle=%d)\n', cfg.robotIP, apiHandle);

% onCleanup: bei Fehler oder CTRL+C sicher stoppen und trennen
cleanupObj = onCleanup(@() safeShutdown(apiHandle, cfg.nJoints)); %#ok<NASGU>

%% =========================
%  INITIALES FEEDBACK PRUEFEN
%  =========================
fprintf('== Sensor-Feedback pruefen ==\n');

[errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
if errCode ~= 0
    error('Sensor-Feedback fehlgeschlagen (errorCode=%d).', errCode);
end

q0 = extractJointPositions(actuatorFb, cfg.nJoints);
fprintf('Aktuelle Gelenkwinkel [deg]:\n');
for j = 1:cfg.nJoints
    fprintf('  Joint %d: %.2f deg\n', j, q0(j));
end

%% =========================
%  OPTIONAL: IN RETRACT-POSITION FAHREN
%  =========================
if cfg.startFromRetract
    fprintf('\n== Fahre in Retract-Position ==\n');
    retractAngles = [360, 340, 180, 214, 0, 310, 90];

    % Constraint: 0=no_constraint, 1=duration, 2=speed
    retractConstraint = int32(0);  % Default-Speed
    retractSpeed      = 0;
    retractDuration   = 0;

    input('ENTER druecken, um in Retract zu fahren (oder CTRL+C zum Abbrechen)... ', 's');

    errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
        retractConstraint, retractSpeed, retractDuration, retractAngles);
    if errCode ~= 0
        error('Retract-Kommando fehlgeschlagen (errorCode=%d).', errCode);
    end

    fprintf('Retract-Kommando gesendet. Warte bis Roboter steht...\n');
    pause(cfg.waitAfterRetract_s);

    % Feedback nach Retract
    [errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
    if errCode == 0
        q0 = extractJointPositions(actuatorFb, cfg.nJoints);
        fprintf('Position nach Retract [deg]:\n');
        for j = 1:cfg.nJoints
            fprintf('  Joint %d: %.2f deg\n', j, q0(j));
        end
    end
end

%% =========================
%  VELOCITY-TEST SEQUENZ
%  =========================
fprintf('\n========================================\n');
fprintf('  SICHERHEITSHINWEIS\n');
fprintf('========================================\n');
fprintf(' - Arbeitsraum FREI? E-STOP bereit?\n');
fprintf(' - Testgeschwindigkeit: %.1f deg/s\n', cfg.testSpeed_degPerSec);
fprintf(' - Saettigung bei: %.1f deg/s\n', cfg.dqMax_degPerSec);
fprintf(' - Testdauer pro Richtung: %.1f s\n', cfg.testDuration_s);
fprintf(' - Abbruch mit CTRL+C stoppt automatisch (Zero-Velocity).\n');
fprintf('========================================\n\n');

input('ENTER druecken, um den Velocity-Test zu starten... ', 's');

% Log-Strukturen
log.t      = [];
log.q      = [];
log.dq_cmd = [];
log.note   = {};

nJ = cfg.nJoints;
dt = 1 / cfg.rateHz;
r  = rateControl(cfg.rateHz);

for j = 1:nJ
    fprintf('\n--- Joint %d / %d ---\n', j, nJ);

    dirs = +1;
    if cfg.testBothDirections
        dirs = [+1, -1]; %#ok<NBRAK>
    end

    for d = dirs
        dirStr = directionString(d);
        fprintf('Richtung: %s (%.1f deg/s)\n', dirStr, d * cfg.testSpeed_degPerSec);
        input('ENTER zum Ausfuehren (oder CTRL+C zum Abbrechen)... ', 's');

        % Velocity-Kommando aufbauen: nur Joint j bewegen
        dq_cmd = zeros(1, nJ);
        dq_cmd(j) = d * cfg.testSpeed_degPerSec;

        % Saettigung anwenden
        dq_cmd = saturate(dq_cmd, cfg.dqMax_degPerSec);

        reset(r);

        % === AKTIVE PHASE: Velocity-Kommandos senden ===
        tStart = tic;
        while toc(tStart) < cfg.testDuration_s
            % Feedback lesen (Watchdog)
            [errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
            if errCode ~= 0
                warning('Feedback-Fehler (errorCode=%d)! Stoppe sofort.', errCode);
                sendZeroVelocity(apiHandle, nJ, cfg.speedCmdDuration);
                error('Feedback waehrend Velocity-Test verloren.');
            end
            q_now = extractJointPositions(actuatorFb, nJ);

            % Velocity-Kommando senden
            errCode = kortexApiMexInterface('SendJointSpeedCommand', ...
                apiHandle, cfg.speedCmdDuration, dq_cmd, uint32(nJ));
            if errCode ~= 0
                warning('SendJointSpeedCommand fehlgeschlagen (errorCode=%d)! Stoppe.', errCode);
                sendZeroVelocity(apiHandle, nJ, cfg.speedCmdDuration);
                error('Velocity-Kommando fehlgeschlagen.');
            end

            % Loggen
            log.t(end+1, 1)      = toc(tStart);           %#ok<SAGROW>
            log.q(end+1, :)      = q_now;                  %#ok<SAGROW>
            log.dq_cmd(end+1, :) = dq_cmd;                 %#ok<SAGROW>
            log.note{end+1, 1}   = sprintf('J%d %s', j, dirStr); %#ok<SAGROW>

            waitfor(r);
        end

        % === STOP-PHASE: Nullgeschwindigkeit senden ===
        fprintf('Stop-Phase...\n');
        tStop = tic;
        while toc(tStop) < cfg.restDuration_s
            sendZeroVelocity(apiHandle, nJ, cfg.speedCmdDuration);

            [errCode, ~, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
            if errCode == 0
                q_now = extractJointPositions(actuatorFb, nJ);
                log.t(end+1, 1)      = toc(tStart) + toc(tStop); %#ok<SAGROW>
                log.q(end+1, :)      = q_now;                     %#ok<SAGROW>
                log.dq_cmd(end+1, :) = zeros(1, nJ);               %#ok<SAGROW>
                log.note{end+1, 1}   = sprintf('J%d STOP', j);    %#ok<SAGROW>
            end

            waitfor(r);
        end

        % Sicherheitshalber nochmal explizit stoppen
        sendZeroVelocity(apiHandle, nJ, cfg.speedCmdDuration);

        fprintf('OK: Joint %d Richtung %s abgeschlossen.\n', j, dirStr);
    end
end

% Finales Stop
sendZeroVelocity(apiHandle, nJ, cfg.speedCmdDuration);
fprintf('\n== Velocity-Test abgeschlossen. Roboter gestoppt. ==\n');

%% =========================
%  OPTIONAL: ZURUECK ZU RETRACT
%  =========================
fprintf('\n== Zurueck in Retract-Position ==\n');
input('ENTER zum Zurueckfahren (oder CTRL+C zum Ueberspringen)... ', 's');

retractAngles = [360, 340, 180, 214, 0, 310, 90];
errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, ...
    int32(0), 0, 0, retractAngles);
if errCode == 0
    fprintf('Retract-Kommando gesendet. Warte...\n');
    pause(cfg.waitAfterRetract_s);
else
    warning('Retract-Kommando fehlgeschlagen (errorCode=%d).', errCode);
end

%% =========================
%  LOG SPEICHERN + PLOT
%  =========================
if cfg.saveLog && ~isempty(log.t)
    save(cfg.logFile, 'cfg', 'log');
    fprintf('Log gespeichert: %s\n', cfg.logFile);
end

if ~isempty(log.t)
    figure('Name', 'Kinova Velocity Test');

    subplot(2,1,1);
    plot(log.t, log.dq_cmd);
    grid on; xlabel('t [s]'); ylabel('dq\_cmd [deg/s]');
    title('Kommandierte Gelenkgeschwindigkeiten');
    legend(compose("J%d", 1:nJ), 'Location', 'best');

    subplot(2,1,2);
    plot(log.t, log.q);
    grid on; xlabel('t [s]'); ylabel('q [deg]');
    title('Gemessene Gelenkwinkel');
    legend(compose("J%d", 1:nJ), 'Location', 'best');
end

%% =========================
%  VERBINDUNG TRENNEN
%  =========================
fprintf('\n== Verbindung trennen ==\n');
errCode = kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
if errCode == 0
    fprintf('Verbindung getrennt.\n');
end
clear;

%% ========================================================================
%  LOKALE FUNKTIONEN
%  ========================================================================

function safeShutdown(apiHandle, nJ)
% Wird bei Script-Ende, Fehler oder CTRL+C automatisch ausgefuehrt.
% Sendet Null-Geschwindigkeit und trennt die Verbindung.
    try
        fprintf('\n[safeShutdown] Sende Zero-Velocity...\n');
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, 0, zeros(1, nJ), uint32(nJ));
        pause(0.05);
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, 0, zeros(1, nJ), uint32(nJ));
    catch
        % Best effort
    end
    try
        % Zusaetzlich: StopAction als Sicherheitsnetz
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
% Sendet Null-Geschwindigkeit an alle Gelenke
    try
        kortexApiMexInterface('SendJointSpeedCommand', ...
            apiHandle, duration, zeros(1, nJ), uint32(nJ));
    catch
    end
end

function q = extractJointPositions(actuatorFb, nJ)
% Extrahiert Gelenkwinkel [deg] aus dem Actuator-Feedback.
%
% actuatorFb kann sein:
%   a) Ein einzelner Struct mit 'position' als 1xN Vektor
%   b) Ein Struct-Array (1 pro Joint) mit skalarem 'position'-Feld
    q = zeros(1, nJ);

    % Fall a): Einzelner Struct, position ist ein Vektor
    if numel(actuatorFb) == 1 && isfield(actuatorFb, 'position') ...
            && numel(actuatorFb.position) >= nJ
        q = double(actuatorFb.position(1:nJ));
        return;
    end

    % Fall b): Struct-Array (1 Element pro Joint)
    for i = 1:min(nJ, numel(actuatorFb))
        if isfield(actuatorFb, 'position')
            q(i) = double(actuatorFb(i).position);
        end
    end
end

function y = saturate(x, lim)
% Begrenzt jeden Wert von x auf [-lim, +lim]
    y = max(min(x, lim), -lim);
end

function s = directionString(d)
    if d > 0, s = '+'; else, s = '-'; end
end
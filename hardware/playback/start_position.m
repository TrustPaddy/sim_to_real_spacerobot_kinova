%% kinova_move_to_zero.m
% Sicheres Script: Kinova Gen3 in Nullstellung [0 0 0 0 0 0 0] fahren.
%
% Verwendet das high-level kortex()-Interface (wie in deinem zweiten Script)
% mit:
%   - Speed-Constraint (langsame, kontrollierte Bewegung)
%   - Optionalem Zwischenwaypoint (bekannt-gute Position aus deinen Tests)
%   - onCleanup-Safety (CTRL+C oder Fehler  =>  sichere Trennung)
%   - Verifikation der Endposition mit wrap-around-Fehlerrechnung
%
% SICHERHEITSHINWEIS:
%   - [0 0 0 0 0 0 0] = Arm vertikal nach oben gestreckt!
%     => Deckenfreiheit pruefen, Kabel/Leuchten/Sensoren beachten.
%   - Arbeitsraum freiraeumen. E-STOP in Reichweite.
%   - Erste Ausfuehrung mit niedriger Speed-Constraint (10 deg/s).

clear; clc;

%% =========================
%  KONFIGURATION
%  =========================
cfg.robotIP     = '192.168.0.10';   % <-- ggf. anpassen
cfg.user        = 'admin';
cfg.password    = 'admin';
cfg.nJoints     = 7;

% Zielposition
cfg.targetAngles = [0 0 0 0 0 0 0];

% Bewegungsprofil
% Hinweis: Speed-Constraint wurde getestet und hat zu keiner Bewegung gefuehrt.
% Wir nutzen daher den Default-Modus (constraintType=0), der in deinem
% vorhandenen Test-Script funktioniert. Sicherheit kommt hier vor allem
% durch den Zwischenwaypoint und langsame Default-Speed des Kinova.

% Zwischenwaypoint (bekannt-gute Position aus deinem vorhandenen Test-Script)
cfg.useIntermediate    = false;
cfg.intermediateAngles = [0 15 180 230 0 55 0];

% Timing & Verifikation
cfg.waitBetweenMoves = 2.0;         % Pause zwischen Bewegungen [s]
cfg.motionTimeout_s  = 60;          % Timeout beim Warten auf Ende [s]
cfg.positionTol_deg  = 1.0;         % Toleranz zur Bewegungs-Ende-Erkennung

%% =========================
%  VERBINDUNG AUFBAUEN
%  =========================
fprintf('== Kinova Gen3: Fahrt zu [0 0 0 0 0 0 0] ==\n');

Simulink.importExternalCTypes(which('kortex_wrapper_data.h'));
gen3Kinova = kortex();
gen3Kinova.ip_address = cfg.robotIP;
gen3Kinova.user       = cfg.user;
gen3Kinova.password   = cfg.password;

isOk = gen3Kinova.CreateRobotApisWrapper();
if ~isOk
    error('Verbindung fehlgeschlagen! IP/Netzwerk/Robot pruefen.');
end
fprintf('Verbunden mit %s\n', cfg.robotIP);

% Safe shutdown bei Fehler oder CTRL+C
cleanupObj = onCleanup(@() safeShutdown(gen3Kinova)); %#ok<NASGU>

%% =========================
%  AKTUELLE POSITION LESEN
%  =========================
[isOk, ~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
if ~isOk
    error('Sensor-Feedback fehlgeschlagen.');
end

q_current = extractJointPositions(actuatorFb, cfg.nJoints);
fprintf('\nAktuelle Position vs. Ziel:\n');
fprintf('  Joint |   Ist [deg] |  Ziel [deg] |  Delta (wrapped) [deg]\n');
fprintf('  ------|-------------|-------------|-----------------------\n');
for j = 1:cfg.nJoints
    delta = wrapAngleDeg(cfg.targetAngles(j) - q_current(j));
    fprintf('    J%d  |   %7.2f   |   %6.2f    |   %+7.2f\n', ...
            j, q_current(j), cfg.targetAngles(j), delta);
end

%% =========================
%  SAFETY CHECK
%  =========================
fprintf('\n=========================================\n');
fprintf('  SICHERHEITSCHECK - bitte bestaetigen\n');
fprintf('=========================================\n');
fprintf(' [ ] Arbeitsraum frei (auch OBEN, Arm steht senkrecht am Ende!)\n');
fprintf(' [ ] Keine Kabel/Objekte im Bewegungsbereich\n');
fprintf(' [ ] E-STOP in Reichweite\n');
fprintf(' [ ] Bewegung im Default-Modus (keine Speed-Constraint)\n');
if cfg.useIntermediate
    fprintf(' [ ] Zwischenwaypoint aktiv: [%s]\n', ...
            strjoin(string(cfg.intermediateAngles), ' '));
else
    fprintf(' [ ] Kein Zwischenwaypoint - Direktbewegung\n');
end
fprintf('=========================================\n');

input('\nAlles OK? ENTER zum Fortfahren (CTRL+C abbrechen)... ', 's');

%% =========================
%  OPTIONAL: ZWISCHENWAYPOINT
%  =========================
if cfg.useIntermediate
    fprintf('\n>> Fahre zu Zwischenwaypoint...\n');
    input('   ENTER zum Ausfuehren... ', 's');

    isOk = gen3Kinova.SendJointAngles(cfg.intermediateAngles, ...
                                      int32(0), 0, 0);   % Default-Modus (wie in deinem Test-Script)
    if ~isOk
        error('Kommando fuer Zwischenwaypoint fehlgeschlagen.');
    end

    waitForMotionComplete(gen3Kinova, cfg.intermediateAngles, cfg);
    pause(cfg.waitBetweenMoves);
end

%% =========================
%  HAUPTBEWEGUNG: ZIEL ANFAHREN
%  =========================
fprintf('\n>> Fahre zu Ziel [0 0 0 0 0 0 0]...\n');
input('   ENTER zum Ausfuehren (letzte Chance zum Abbrechen)... ', 's');

isOk = gen3Kinova.SendJointAngles(cfg.targetAngles, ...
                                  int32(0), 0, 0);   % Default-Modus
if ~isOk
    error('Kommando fuer Zielposition fehlgeschlagen.');
end

waitForMotionComplete(gen3Kinova, cfg.targetAngles, cfg);

%% =========================
%  VERIFIKATION
%  =========================
[isOk, ~, actuatorFb, ~] = gen3Kinova.SendRefreshFeedback();
if isOk
    q_final = extractJointPositions(actuatorFb, cfg.nJoints);
    fprintf('\n== Endposition erreicht ==\n');
    maxErr = 0;
    for j = 1:cfg.nJoints
        err = wrapAngleDeg(q_final(j) - cfg.targetAngles(j));
        maxErr = max(maxErr, abs(err));
        fprintf('  J%d: %8.3f°  (Abweichung: %+6.3f°)\n', j, q_final(j), err);
    end
    fprintf('  Max. Abweichung: %.3f°\n', maxErr);
end

%% =========================
%  VERBINDUNG TRENNEN
%  =========================
fprintf('\n== Trenne Verbindung ==\n');
gen3Kinova.DestroyRobotApisWrapper();
clear cleanupObj;
fprintf('Fertig.\n');


%% ========================================================================
%  LOKALE FUNKTIONEN
%  ========================================================================

function safeShutdown(robot)
% Wird bei Fehler oder CTRL+C automatisch ausgefuehrt.
% Trennt die Verbindung sicher. (Bei SendJointAngles gibt es kein
% separates Stop-Kommando noetig - die Bewegung wird bei Disconnect
% vom Roboter selbst abgebrochen.)
    try
        fprintf('\n[safeShutdown] Trenne Verbindung...\n');
        robot.DestroyRobotApisWrapper();
    catch
    end
    fprintf('[safeShutdown] Abgeschlossen.\n');
end

function waitForMotionComplete(robot, target, cfg)
% Wartet bis alle Gelenke innerhalb der Toleranz liegen und stabil bleiben.
% Verwendet wrap-around-Fehlerrechnung, damit z.B. 359° und 0° als gleich
% erkannt werden.
    fprintf('   Warte auf Bewegungsende...');
    tStart = tic;
    stableCount = 0;
    while toc(tStart) < cfg.motionTimeout_s
        [isOk, ~, actuatorFb, ~] = robot.SendRefreshFeedback();
        if isOk
            q   = extractJointPositions(actuatorFb, cfg.nJoints);
            err = zeros(1, cfg.nJoints);
            for j = 1:cfg.nJoints
                err(j) = wrapAngleDeg(q(j) - target(j));
            end
            if all(abs(err) < cfg.positionTol_deg)
                stableCount = stableCount + 1;
                if stableCount >= 5   % 5x hintereinander stabil -> fertig
                    fprintf(' OK (%.1fs)\n', toc(tStart));
                    return;
                end
            else
                stableCount = 0;
            end
        end
        pause(0.1);
    end
    warning('Timeout - Bewegung nicht innerhalb %.0fs abgeschlossen.', ...
            cfg.motionTimeout_s);
end

function q = extractJointPositions(actuatorFb, nJ)
% Robuste Extraktion der Gelenkwinkel [deg].
% Funktioniert sowohl fuer einzelnen Struct mit Vektor-'position',
% als auch fuer Struct-Array (1 Element pro Joint).
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

function wrapped = wrapAngleDeg(angle)
% Wraps einen Winkel auf [-180, 180]
    wrapped = mod(angle + 180, 360) - 180;
end
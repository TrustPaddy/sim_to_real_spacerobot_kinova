%% dq_cmd_to_ee_trajectory_fk.m
% Aus dq_cmd.mat:
%   1. dq laden
%   2. dq ueber die Zeit zu q integrieren
%   3. mit Forward-Kinematics die Endeffektor-Trajektorie berechnen
%   4. EE-Bahn in xz-Ebene plotten
%
% Zweck:
%   Damit kannst du pruefen, welche kartesische Bahn deine gespeicherten
%   Gelenkgeschwindigkeiten rein kinematisch erzeugen wuerden.
%
% Erwarteter Inhalt von dq_cmd.mat:
%   - dq : Nx7 Gelenkgeschwindigkeiten, standardmaessig rad/s
%   - t  : Nx1 Zeitvektor in Sekunden
%
% Wichtig:
%   Wenn dq_cmd aus dem Training mit SpaceKinova.urdf stammt, dann fuer die
%   erste Analyse auch SpaceKinova.urdf verwenden. Nur so vergleichst du im
%   gleichen Frame wie im Training.

clear; clc; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg.trajFile   = sk_path("data", "dq_cmd", "dq_cmd_non-singular.mat");
cfg.dqVarName  = "dq";
cfg.tVarName   = "t";
cfg.dqUnit     = "rad/s";          % "rad/s" oder "deg/s"
cfg.nJoints    = 7;

% Startkonfiguration der Simulation/Hardware.
% Bei dir war Training offenbar mit [0 0 0 0 0 0 0].
%cfg.q0_deg = [0 0 0 0 0 0 0];
cfg.q0_deg = [0 15 180 230 0 55 90];


% Integration
cfg.wrapToPiAfterIntegration = false;

% Playback-Dauer optional begrenzen.
% inf = komplette Datei verwenden.
cfg.maxDuration_s = inf;           % z.B. 8.5

% URDF-Auswahl
% Variante A: SpaceKinova, passend zum Training
% cfg.urdfFile   = sk_path("robot", "SpaceKinova.urdf");
% cfg.eeBodyName = "kinova_end_effector_link";

% Variante B: echter Standard-Kinova, zum Vergleich ggf. einkommentieren
cfg.urdfFile   = sk_path("robot", "GEN3-7DOF-VISION_ARM_URDF_V12.urdf");
cfg.eeBodyName = "end_effector_link";

% Tool-Offset im EE-lokalen Frame [m].
% Fuer Vergleich mit Trainings-Flange meistens [0;0;0].
% Fuer Vergleich mit realem Tool-Tip ggf. [0;0;0.115].
cfg.toolOffset = [0; 0; 0];

% Optional: Referenz-Halbkreis aus deinem Training/Deployment plotten
cfg.plotReference = true;
cfg.r      = 0.2;
%cfg.center = [0.0, -0.025, 1.187 - cfg.r];
cfg.center = [0.497, -0.005, 0.436 + cfg.r];
cfg.yConst = 0;

% Speichern
cfg.saveResult = true;
cfg.outFile    = sk_path("data", "dq_cmd", "dq_cmd_fk_result.mat");

%% =========================
%  DATEN LADEN
%  =========================
fprintf("== Lade %s ==\n", cfg.trajFile);
S = load(cfg.trajFile);

assert(isfield(S, cfg.dqVarName), 'Variable "%s" nicht gefunden.', cfg.dqVarName);
assert(isfield(S, cfg.tVarName),  'Variable "%s" nicht gefunden.', cfg.tVarName);

dq = S.(cfg.dqVarName);
t  = S.(cfg.tVarName);
t  = t(:);

% Dimensionen pruefen
if size(dq,2) ~= cfg.nJoints && size(dq,1) == cfg.nJoints
    dq = dq.';
end

assert(size(dq,2) == cfg.nJoints, ...
    "dq muss Nx%d sein. Aktuelle Groesse: %dx%d.", ...
    cfg.nJoints, size(dq,1), size(dq,2));

assert(size(dq,1) == numel(t), ...
    "Laenge von t (%d) passt nicht zu dq (%d).", numel(t), size(dq,1));

% Zeit bei 0 starten
t = t - t(1);

% Optional auf Dauer begrenzen
if isfinite(cfg.maxDuration_s)
    idx = t <= cfg.maxDuration_s + 1e-12;
    t = t(idx);
    dq = dq(idx,:);
end

% Einheiten nach rad/s
switch lower(cfg.dqUnit)
    case "rad/s"
        dq_rad = dq;
    case "deg/s"
        dq_rad = deg2rad(dq);
    otherwise
        error("Unbekannte dqUnit: %s. Nutze 'rad/s' oder 'deg/s'.", cfg.dqUnit);
end

N = size(dq_rad,1);
assert(N >= 2, "Trajektorie hat zu wenige Samples.");

dt = diff(t);
fprintf("Samples         : %d\n", N);
fprintf("Dauer t(end)    : %.6f s\n", t(end));
fprintf("dt median       : %.6f s\n", median(dt));
fprintf("dt min/max      : %.6f / %.6f s\n", min(dt), max(dt));
fprintf("max |dq| rad/s  : %.6f\n", max(abs(dq_rad), [], "all"));
fprintf("max |dq| deg/s  : %.6f\n", max(abs(rad2deg(dq_rad)), [], "all"));

%% =========================
%  dq ZU q INTEGRIEREN
%  =========================
fprintf("\n== Integriere dq zu q ==\n");

q_rad = zeros(N, cfg.nJoints);
q_rad(1,:) = deg2rad(cfg.q0_deg(:).');

for k = 2:N
    dt_k = t(k) - t(k-1);

    % Explizite Euler-Integration:
    % q(k) = q(k-1) + dq(k-1)*dt
    q_rad(k,:) = q_rad(k-1,:) + dq_rad(k-1,:) * dt_k;

    if cfg.wrapToPiAfterIntegration
        q_rad(k,:) = wrapToPiLocal(q_rad(k,:));
    end
end

q_deg = rad2deg(q_rad);

fprintf("Start q [deg]: [%s]\n", join(string(round(q_deg(1,:),4)), " "));
fprintf("Ende  q [deg]: [%s]\n", join(string(round(q_deg(end,:),4)), " "));
fprintf("Delta q [deg]: [%s]\n", join(string(round(q_deg(end,:) - q_deg(1,:),4)), " "));

%% =========================
%  ROBOTER LADEN UND FK BERECHNEN
%  =========================
fprintf("\n== Lade URDF und berechne FK ==\n");
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);

robot = importrobot(cfg.urdfFile);
robot.DataFormat = "row";

% Body-Name pruefen
bodyNames = string(robot.BodyNames);
if ~any(bodyNames == string(cfg.eeBodyName))
    fprintf("\nVerfuegbare BodyNames:\n");
    disp(bodyNames.');
    error("eeBodyName '%s' nicht in URDF gefunden.", cfg.eeBodyName);
end

ee_flange = zeros(N,3);
ee_tool   = zeros(N,3);

for k = 1:N
    T = getTransform(robot, q_rad(k,:), char(cfg.eeBodyName));

    p_flange = T(1:3,4);
    R        = T(1:3,1:3);
    p_tool   = p_flange + R * cfg.toolOffset(:);

    ee_flange(k,:) = p_flange.';
    ee_tool(k,:)   = p_tool.';
end

fprintf("URDF       : %s\n", cfg.urdfFile);
fprintf("EE Body    : %s\n", cfg.eeBodyName);
fprintf("ToolOffset : [%.4f %.4f %.4f] m\n", cfg.toolOffset(1), cfg.toolOffset(2), cfg.toolOffset(3));
fprintf("Start EE flange [m]: [%.4f %.4f %.4f]\n", ee_flange(1,1), ee_flange(1,2), ee_flange(1,3));
fprintf("Ende  EE flange [m]: [%.4f %.4f %.4f]\n", ee_flange(end,1), ee_flange(end,2), ee_flange(end,3));
fprintf("Start EE tool   [m]: [%.4f %.4f %.4f]\n", ee_tool(1,1), ee_tool(1,2), ee_tool(1,3));
fprintf("Ende  EE tool   [m]: [%.4f %.4f %.4f]\n", ee_tool(end,1), ee_tool(end,2), ee_tool(end,3));

%% =========================
%  REFERENZTRAJEKTORIE
%  =========================
if cfg.plotReference
    omega = pi / max(t(end), eps);
    x_ref = cfg.center(1) + cfg.r * sin(omega * t);
    y_ref = cfg.center(2) + cfg.yConst * t;
    z_ref = cfg.center(3) - cfg.r * cos(omega * t);
    traj_ref = [x_ref y_ref z_ref];

    err_flange = vecnorm(ee_flange - traj_ref, 2, 2);
    err_tool   = vecnorm(ee_tool   - traj_ref, 2, 2);

    fprintf("\n== Vergleich mit Referenz ==\n");
    fprintf("Mittlerer Fehler Flange: %.4f m\n", mean(err_flange));
    fprintf("Max Fehler Flange      : %.4f m\n", max(err_flange));
    fprintf("Mittlerer Fehler Tool  : %.4f m\n", mean(err_tool));
    fprintf("Max Fehler Tool        : %.4f m\n", max(err_tool));
else
    traj_ref = [];
    err_flange = [];
    err_tool = [];
end

%% =========================
%  PLOTS
%  =========================
fprintf("\n== Erzeuge Plots ==\n");

% Gelenkgeschwindigkeiten
figure("Name", "dq_cmd");
plot(t, rad2deg(dq_rad), "LineWidth", 1.1);
grid on;
xlabel("t [s]");
ylabel("dq [deg/s]");
title("Geladene Gelenkgeschwindigkeiten dq\_cmd");
legend(compose("J%d", 1:cfg.nJoints), "Location", "best");

% Integrierte Gelenkwinkel
figure("Name", "Integrierte Gelenkwinkel q");
plot(t, q_deg, "LineWidth", 1.1);
grid on;
xlabel("t [s]");
ylabel("q [deg]");
title("Aus dq integrierte Gelenkwinkel");
legend(compose("J%d", 1:cfg.nJoints), "Location", "best");

% EE xz
figure("Name", "EE-Trajektorie aus integrierter dq via FK: xz");
plot(ee_flange(:,1), ee_flange(:,3), "b-", "LineWidth", 1.6); hold on;
plot(ee_tool(:,1),   ee_tool(:,3),   "m-", "LineWidth", 1.2);

if cfg.plotReference
    plot(traj_ref(:,1), traj_ref(:,3), "r--", "LineWidth", 1.5);
end

plot(ee_flange(1,1), ee_flange(1,3), "bo", "MarkerFaceColor", "b");
plot(ee_flange(end,1), ee_flange(end,3), "bx", "LineWidth", 2, "MarkerSize", 10);

if cfg.plotReference
    plot(traj_ref(1,1), traj_ref(1,3), "ks", "MarkerFaceColor", "k");
end

grid on; axis equal;
xlabel("x [m]");
ylabel("z [m]");
title("EE-Trajektorie aus integrierter dq via FK, xz-Ebene");

if cfg.plotReference
    legend("EE Flange", "EE Tool", "Soll Referenz", ...
           "Start Flange", "Ende Flange", "Start Soll", ...
           "Location", "best");
else
    legend("EE Flange", "EE Tool", "Start Flange", "Ende Flange", ...
           "Location", "best");
end

% EE 3D
figure("Name", "EE-Trajektorie 3D");
plot3(ee_flange(:,1), ee_flange(:,2), ee_flange(:,3), "b-", "LineWidth", 1.6); hold on;
plot3(ee_tool(:,1),   ee_tool(:,2),   ee_tool(:,3),   "m-", "LineWidth", 1.2);

if cfg.plotReference
    plot3(traj_ref(:,1), traj_ref(:,2), traj_ref(:,3), "r--", "LineWidth", 1.5);
end

grid on; axis equal;
xlabel("x [m]");
ylabel("y [m]");
zlabel("z [m]");
title("EE-Trajektorie 3D aus integrierter dq via FK");

if cfg.plotReference
    legend("EE Flange", "EE Tool", "Soll Referenz", "Location", "best");
else
    legend("EE Flange", "EE Tool", "Location", "best");
end

% Fehlerplot
if cfg.plotReference
    figure("Name", "Kartesischer Fehler zur Referenz");
    plot(t, err_flange, "b-", "LineWidth", 1.3); hold on;
    plot(t, err_tool,   "m-", "LineWidth", 1.3);
    grid on;
    xlabel("t [s]");
    ylabel("||p - p_{ref}|| [m]");
    title("Kartesischer Abstand zur Referenztrajektorie");
    legend("Flange", "Tool", "Location", "best");
end

%% =========================
%  SPEICHERN
%  =========================
result.cfg        = cfg;
result.t          = t;
result.dq_rad     = dq_rad;
result.dq_degps   = rad2deg(dq_rad);
result.q_rad      = q_rad;
result.q_deg      = q_deg;
result.ee_flange  = ee_flange;
result.ee_tool    = ee_tool;
result.traj_ref   = traj_ref;
result.err_flange = err_flange;
result.err_tool   = err_tool;

if cfg.saveResult
    save(cfg.outFile, "result");
    fprintf("\nErgebnis gespeichert: %s\n", cfg.outFile);
end

fprintf("\nFertig.\n");

%% =========================
%  LOKALE FUNKTION
%  =========================
function y = wrapToPiLocal(x)
    y = mod(x + pi, 2*pi) - pi;
end

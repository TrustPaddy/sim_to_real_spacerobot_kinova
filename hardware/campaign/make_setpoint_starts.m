function S = make_setpoint_starts(varargin)
%MAKE_SETPOINT_STARTS  Feste Liste von Startposen fuer die Set-Point-Reihe (R1.9).
%
%   S = MAKE_SETPOINT_STARTS() berechnet die Liste mit den Standardwerten und
%   speichert sie in hardware/campaign/setpoint_starts.mat und .csv sowie eine
%   Uebersicht setpoint_starts_overview.png.
%   S = MAKE_SETPOINT_STARTS('save', false) berechnet nur.
%
%   Das Skript laeuft ohne Roboter und wird einmal auf dem Desktop ausgefuehrt. Die
%   gespeicherte Datei ist die Definition der Reihe, nicht dieses Skript.
%
%   Ziel ist das nominale Trainingsziel [0.479 -0.005 1.136] m im URDF-Frame
%   (SpaceKinova_Point_CDR.m). Fuer jeden Startabstand d0 (Abstand des
%   Endeffektors zum Ziel) werden zwei Gelenkkonfigurationen gewaehlt. Die
%   Kandidaten stammen aus einer Stichprobe mit der Startverteilung der letzten
%   Trainingsphase (Anker + 40 deg Rauschen oder gleichverteilt in den Grenzen).
%   Dazu kommt S00, der Trainingsanker selbst.
%
%   Filter (offline, ohne Kenntnis der Zelle):
%     - Gelenke mindestens marginDeg von der engeren Grenze aus Deploy-Skript und
%       Gen3-Hardware entfernt, Endlosgelenke hoechstens 170 deg vom Anker
%     - Unterarm, Handgelenk, Endeffektor und Greiferpunkte mindestens zMin
%       (0.15 m) ueber der Montageflaeche
%     - keine Kollision laut URDF-Kollisionsgeometrie (checkCollision)
%     - kleinster Singulaerwert der Positions-Jacobi mindestens sigmaMin
%     - die lineare Gelenkbahn Anker -> Start erfuellt Hoehe und Kollision
%   Unter den gueltigen Kandidaten einer Stufe gewinnt der mit der kleinsten
%   Gelenkabweichung vom Anker. Der zweite Start einer Stufe liegt in einer anderen
%   Richtung vom Ziel aus (Winkel mindestens minDirAngleDeg).
%
%   Hindernisse der Zelle (Satelliten-Mockup, zweiter Roboter, Linearachse) kennt
%   das Skript nicht. Jede Pose muss deshalb vor der Messreihe im Labor mit
%   check_setpoint_starts angefahren und freigegeben werden.

p = inputParser;
p.addParameter('levels', [0.10 0.20 0.35 0.50 0.70 0.90 1.10]);
p.addParameter('perLevel', 2);
p.addParameter('levelTol', 0.025);
p.addParameter('nSamples', 60000);
p.addParameter('seed', 2026);
p.addParameter('marginDeg', 20);
p.addParameter('zMin', 0.15);
p.addParameter('sigmaMin', 0.05);
p.addParameter('minDirAngleDeg', 60);
p.addParameter('save', true);
p.parse(varargin{:});
o = p.Results;

rbt = importrobot(sk_path('robot', 'SpaceKinova.urdf'));
rbt.DataFormat = 'row';
ee = 'kinova_end_effector_link';
F = kortex_frames(rbt);

anchor = deg2rad([0 15 180 -130 0 55 90]);
target = [0.479 -0.005 1.136];
% Grenzen: engere aus Deploy-Skript (qLim) und Gen3-Datenblatt (J2, J4, J6)
qLimDeploy = [2*pi 2.41 2*pi 2.66 2.23 2.01 2*pi];
qLimHw     = deg2rad([inf 128.9 inf 147.8 inf 120.3 inf]);
qLim   = min(qLimDeploy, qLimHw);
cont   = logical([1 0 1 0 0 0 1]);            % Endlosgelenke (J5 hat im Deploy eine Grenze)
margin = deg2rad(o.marginDeg);
qLo = -qLim + margin;  qHi = qLim - margin;
qLo(cont) = anchor(cont) - deg2rad(170);  qHi(cont) = anchor(cont) + deg2rad(170);

%% Ziel: IK wie im Training und eine Loesung innerhalb der Grenzen fuer den Anfahrtest
rng(0, 'twister');
ik = inverseKinematics('RigidBodyTree', rbt);
[qIkTrain, ~] = ik(ee, [eye(3) target.'; 0 0 0 1], [0.05 0.05 0.05 1 1 1], homeConfiguration(rbt));
rng(o.seed, 'twister');
lim = zeros(7, 2);
for i = 1:7
    lim(i, :) = rbt.Bodies{i + 1}.Joint.PositionLimits;   % Bodies{1} ist kinova_base_link (fest)
end
qTargetCheck = [];
best = inf;
for i = 1:200
    seed = min(max(anchor + deg2rad(30) * randn(1, 7), lim(:, 1).'), lim(:, 2).');
    [qs, info] = ik(ee, [eye(3) target.'; 0 0 0 1], [0 0 0 1 1 1], seed);
    T = getTransform(rbt, qs, ee);
    if norm(T(1:3, 4).' - target) > 1e-3 || ~strcmp(info.Status, 'success'), continue; end
    qs = unwrap_to(qs, anchor);
    if any(qs < qLo | qs > qHi), continue; end
    [okH, ~] = heights_ok(rbt, qs, F, o.zMin);
    if ~okH || checkCollision(rbt, qs, 'SkippedSelfCollisions', 'parent'), continue; end
    dev = max(abs(rad2deg(qs - anchor)));
    if dev < best, best = dev; qTargetCheck = qs; end
end
assert(~isempty(qTargetCheck), 'Keine gueltige Zielkonfiguration gefunden.');

%% Stichprobe wie Trainingsphase 4
n = o.nSamples;
Q = zeros(n, 7);
useAnchor = rand(n, 1) < 0.5;
Q(useAnchor, :) = anchor + deg2rad(40) * randn(nnz(useAnchor), 7);
trainLo = -[2*pi 2.41 2*pi 2.66 2.23 2.01 2*pi];  trainHi = -trainLo;
m5 = 0.05 * (trainHi - trainLo);
Q(~useAnchor, :) = (trainLo + m5) + (trainHi - trainLo - 2 * m5) .* rand(nnz(~useAnchor), 7);
Q = unwrap_to(Q, anchor);
inLim = all(Q >= qLo & Q <= qHi, 2);
Q = Q(inLim, :);
fprintf('Stichprobe: %d von %d innerhalb der Grenzen\n', size(Q, 1), n);

eePos = zeros(size(Q, 1), 3);
for i = 1:size(Q, 1)
    T = getTransform(rbt, Q(i, :), ee);
    eePos(i, :) = T(1:3, 4).';
end
d0 = vecnorm(eePos - target, 2, 2);
dev = max(abs(rad2deg(Q - anchor)), [], 2);

%% Auswahl je Stufe
starts = struct('id', {}, 'level_m', {}, 'd0_m', {}, 'q_deg', {}, 'ee_urdf', {}, 'ee_real', {}, ...
                'tool_pred_real', {}, 'dir', {}, 'maxDevAnchor_deg', {}, 'minHeight_m', {}, ...
                'sigmaMin', {}, 'dirAngleToFirst_deg', {});
starts(end+1) = describe(rbt, F, 'S00', NaN, anchor, target, anchor, NaN);
id = 0;
for L = o.levels
    cand = find(abs(d0 - L) <= o.levelTol);
    [~, ord] = sort(dev(cand));
    cand = cand(ord);
    chosen = [];
    for j = 1:o.perLevel
        for c = cand.'
            if any(chosen == c), continue; end
            dirC = (eePos(c, :) - target) / d0(c);
            angFirst = NaN;
            if ~isempty(chosen)
                dirA = (eePos(chosen(1), :) - target) / d0(chosen(1));
                angFirst = acosd(max(-1, min(1, dot(dirA, dirC))));
                if angFirst < o.minDirAngleDeg, continue; end
            end
            if ~pose_ok(rbt, Q(c, :), F, o), continue; end
            if ~path_ok(rbt, anchor, Q(c, :), F, o), continue; end
            chosen(end+1) = c; %#ok<AGROW>
            id = id + 1;
            starts(end+1) = describe(rbt, F, sprintf('S%02d', id), L, Q(c, :), target, anchor, angFirst); %#ok<AGROW>
            break;
        end
        if numel(chosen) < j
            warning('make_setpoint_starts:level', 'Stufe %.2f m: nur %d von %d Starts gefunden.', ...
                    L, numel(chosen), o.perLevel);
            break;
        end
    end
end

%% Ausgabe
Tt = getTransform(rbt, qTargetCheck, ee);
S = struct();
S.schema = 'sk_setpoint_starts_v1';
S.meta = struct('generator', 'make_setpoint_starts.m', 'datetime', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
                'params', o, 'anchor_deg', rad2deg(anchor), 'qLo_deg', rad2deg(qLo), 'qHi_deg', rad2deg(qHi), ...
                'nCandidatesInLimits', size(Q, 1), 'kortexToolOffset', F.toolOffset.', 'kortexNote', F.note);
env = campaign_env_meta();
S.meta.gitHash = env.gitHash;  S.meta.gitDirty = env.gitDirty;
S.target = struct('ee_urdf', target, 'ee_real', to_real(F, target), ...
                  'q_ik_training_deg', rad2deg(qIkTrain), 'q_check_deg', rad2deg(qTargetCheck), ...
                  'tool_pred_real_at_check', to_real(F, (Tt(1:3, 4) + Tt(1:3, 1:3) * F.toolOffset).'), ...
                  'note', ['q_ik_training_deg ist die IK-Loesung des Trainings (J6 ausserhalb der ' ...
                           'Deploy-Grenze). q_check_deg ist eine Loesung innerhalb der Grenzen, nur fuer den Anfahrtest.']);
S.starts = starts;

fprintf('\n%-4s %6s %6s %8s %7s %7s  %s\n', 'ID', 'Stufe', 'd0', 'maxDev', 'zMin', 'sMin', 'q [deg]');
for i = 1:numel(starts)
    s = starts(i);
    fprintf('%-4s %6.2f %6.3f %8.1f %7.3f %7.3f  %s\n', s.id, s.level_m, s.d0_m, s.maxDevAnchor_deg, ...
            s.minHeight_m, s.sigmaMin, mat2str(round(s.q_deg, 1)));
end

if o.save
    here = fileparts(mfilename('fullpath'));
    save(fullfile(here, 'setpoint_starts.mat'), 'S', '-v7');
    write_csv(fullfile(here, 'setpoint_starts.csv'), S);
    plot_overview(rbt, S, fullfile(here, 'setpoint_starts_overview.png'));
    plot_map(S, fullfile(here, 'setpoint_starts_map.png'));
    fprintf('Gespeichert: %s\n', fullfile(here, 'setpoint_starts.mat'));
end
end

function plot_map(S, path)
% Draufsicht und Seitenansicht der Endeffektor-Startpunkte im Kortex-Frame
% (Ursprung = Robotersockel, z = Montageflaeche).
P = reshape([S.starts.ee_real], 3, []).';
tg = S.target.ee_real;
fig = figure('Visible', 'off', 'Position', [0 0 1200 520]);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
views = {[1 2], 'x [m]', 'y [m]', 'Draufsicht (Kortex-Frame)'; [1 3], 'x [m]', 'z [m]', 'Seitenansicht'};
for v = 1:2
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    a = views{v, 1};
    plot(ax, 0, 0, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [0.6 0.6 0.6]);
    plot(ax, tg(a(1)), tg(a(2)), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.92 0.41 0.20], 'MarkerEdgeColor', 'k');
    for i = 1:size(P, 1)
        plot(ax, [tg(a(1)) P(i, a(1))], [tg(a(2)) P(i, a(2))], ':', 'Color', [0.6 0.6 0.6]);
        plot(ax, P(i, a(1)), P(i, a(2)), 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.16 0.47 0.84], ...
             'MarkerEdgeColor', 'w');
        text(ax, P(i, a(1)) + 0.02, P(i, a(2)) + 0.02, sprintf('%s (%.2f)', S.starts(i).id, S.starts(i).d0_m), ...
             'FontSize', 8);
    end
    if v == 2, yline(ax, 0, 'k-', 'Montageflaeche'); end
    xlabel(ax, views{v, 2}); ylabel(ax, views{v, 3}); title(ax, views{v, 4});
end
exportgraphics(fig, path, 'Resolution', 120);
close(fig);
end

%% ========================================================================
function q = unwrap_to(q, ref)
q = q + 2*pi * round((ref - q) / (2*pi));
end

function p = to_real(F, pUrdf)
v = F.T_urdf_from_real \ [pUrdf(:); 1];
p = v(1:3).';
end

function [ok, zmin] = heights_ok(rbt, q, F, zMin)
% Hoehe ueber dem Tisch = z im Kortex-Frame. Geprueft werden Handgelenk,
% Endeffektor und Greiferpunkte (der Greifer fehlt in der URDF).
zBase = F.T_urdf_from_real(3, 4);
Tw = getTransform(rbt, q, 'kinova_spherical_wrist_2_link');
Tf = getTransform(rbt, q, 'kinova_forearm_link');
Te = getTransform(rbt, q, 'kinova_end_effector_link');
pts = Te(1:3, 1:3) * F.gripperPoints + Te(1:3, 4);
z = [Tw(3, 4), Tf(3, 4), pts(3, :)] - zBase;
zmin = min(z);
ok = zmin >= zMin;
end

function ok = pose_ok(rbt, q, F, o)
ok = false;
[okH, ~] = heights_ok(rbt, q, F, o.zMin);
if ~okH, return; end
J = geometricJacobian(rbt, q, 'kinova_end_effector_link');
if min(svd(J(4:6, :))) < o.sigmaMin, return; end
if checkCollision(rbt, q, 'SkippedSelfCollisions', 'parent'), return; end
ok = true;
end

function ok = path_ok(rbt, qA, qB, F, o)
ok = false;
for s = linspace(0, 1, 11)
    q = qA + s * (qB - qA);
    if ~heights_ok(rbt, q, F, o.zMin), return; end
    if mod(round(s * 10), 2) == 0 && checkCollision(rbt, q, 'SkippedSelfCollisions', 'parent'), return; end
end
ok = true;
end

function s = describe(rbt, F, id, level, q, target, anchor, angFirst)
T = getTransform(rbt, q, 'kinova_end_effector_link');
J = geometricJacobian(rbt, q, 'kinova_end_effector_link');
e = T(1:3, 4).';
[~, zmin] = heights_ok(rbt, q, F, -inf);
s = struct('id', id, 'level_m', level, 'd0_m', norm(e - target), 'q_deg', rad2deg(q), ...
           'ee_urdf', e, 'ee_real', to_real(F, e), ...
           'tool_pred_real', to_real(F, (T(1:3, 4) + T(1:3, 1:3) * F.toolOffset).'), ...
           'dir', (e - target) / max(norm(e - target), eps), ...
           'maxDevAnchor_deg', max(abs(rad2deg(q - anchor))), 'minHeight_m', zmin, ...
           'sigmaMin', min(svd(J(4:6, :))), 'dirAngleToFirst_deg', angFirst);
end

function write_csv(path, S)
fid = fopen(path, 'w');
c = onCleanup(@() fclose(fid));
fprintf(fid, '# Erzeugt von make_setpoint_starts.m am %s (Git %s). Nicht von Hand aendern.\n', ...
        S.meta.datetime, S.meta.gitHash);
fprintf(fid, '# Ziel URDF [m]: %s, Kortex-Frame (EE-Punkt): %s\n', mat2str(S.target.ee_urdf, 4), ...
        mat2str(S.target.ee_real, 4));
fprintf(fid, 'id,level_m,d0_m,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg,');
fprintf(fid, 'ee_urdf_x,ee_urdf_y,ee_urdf_z,ee_real_x,ee_real_y,ee_real_z,');
fprintf(fid, 'tool_pred_real_x,tool_pred_real_y,tool_pred_real_z,maxDevAnchor_deg,minHeight_m,sigmaMin\n');
for i = 1:numel(S.starts)
    s = S.starts(i);
    fprintf(fid, '%s,%.2f,%.4f,%s,%s,%s,%s,%.1f,%.3f,%.3f\n', s.id, s.level_m, s.d0_m, ...
            strjoin(compose('%.2f', s.q_deg), ','), strjoin(compose('%.4f', s.ee_urdf), ','), ...
            strjoin(compose('%.4f', s.ee_real), ','), strjoin(compose('%.4f', s.tool_pred_real), ','), ...
            s.maxDevAnchor_deg, s.minHeight_m, s.sigmaMin);
end
end

function plot_overview(rbt, S, path)
n = numel(S.starts) + 1;
nc = 4; nr = ceil(n / nc);
fig = figure('Visible', 'off', 'Position', [0 0 1600 400 * nr]);
tl = tiledlayout(fig, nr, nc, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:n
    ax = nexttile(tl);
    if i == 1
        q = deg2rad(S.target.q_check_deg); ttl = 'Z00 (Ziel, Anfahrtest)';
    else
        s = S.starts(i - 1); q = deg2rad(s.q_deg);
        ttl = sprintf('%s: d_0 = %.2f m', s.id, s.d0_m);
    end
    show(rbt, q, 'Parent', ax, 'Frames', 'off', 'PreservePlot', false, 'Visuals', 'on');
    hold(ax, 'on');
    plot3(ax, S.target.ee_urdf(1), S.target.ee_urdf(2), S.target.ee_urdf(3), 'o', ...
          'MarkerSize', 8, 'MarkerFaceColor', [0.92 0.41 0.20], 'MarkerEdgeColor', 'k');
    title(ax, ttl);
    view(ax, 135, 20); axis(ax, [-1.1 1.1 -1.1 1.1 0 1.8]);
end
exportgraphics(fig, path, 'Resolution', 110);
close(fig);
end

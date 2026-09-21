%% analyze_sim2real_gap.m
% Vergleicht gepaarte Sim- und Real-Runs aus runs/ und quantifiziert
% den Sim2Real-Gap unter Closed-Loop Policy-Steuerung.
%
% INPUT:
%   runs/   enthaelt Dateien aus run_logger:
%       run_XXX_<label>.mat   (meta.source = 'real', aus B1)
%       sim_XXX_<label>.mat   (meta.source = 'sim',  aus C2)
%   Paare werden ueber meta.label gematcht.
%
%   Erwartete Felder pro Datei:
%       data.t            Nx1  [s]
%       data.q_measured   Nx7  [deg]   (in Sim: gleichbedeutend mit q_state)
%       data.dq_cmd       Nx7  [deg/s] = action der Policy
%       data.ee_measured  Nx3  [m]     (optional, sonst via FK ergaenzt)
%       data.obs          NxD  [-]     (optional aber empfohlen)
%       data.reward       Nx1  [-]     (optional)
%       meta.label, meta.source, meta.startposeName, meta.seed, ...
%
% OUTPUT:
%   analysis_output/sim2real_*.png/pdf
%   analysis_output/sim2real_summary.csv
%
% Abhaengigkeiten: URDF im MATLAB-Pfad, run_logger-Format.

clear; clc; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg.runDir       = sk_path('data', 'hardware', 'runs');
cfg.outDir       = sk_path('data', 'hardware', 'analysis_output');
cfg.urdfFile     = sk_path('robot', 'GEN3-7DOF-VISION_ARM_URDF_V12.urdf');
cfg.eeBodyName   = 'end_effector_link';
cfg.toolOffset   = [0; 0; 0.115];
cfg.nJoints      = 7;

% Schwelle fuer Divergenz-Zeitpunkt t*
cfg.divEEThresh_m   = 0.05;     % EE-Distanz, ab der Sim/Real als 'auseinander' gilt
cfg.divQThresh_deg  = 5.0;      % Alternativ: max Gelenkdifferenz

% Action-Vergleich: zwei States 'aehnlich' wenn ||obs_sim - obs_real|| < ...
cfg.simStateTol     = 0.10;     % normierte Distanz im Observation-Raum

% Plots
cfg.savePlots    = true;
cfg.plotFormats  = {'png', 'pdf'};

% Filter (Regex auf Label, oder 'all')
cfg.labelFilter  = 'all';

%% =========================
%  VORBEREITUNG
%  =========================
assert(isfolder(cfg.runDir), 'Run-Ordner fehlt: %s', cfg.runDir);
if ~isfolder(cfg.outDir), mkdir(cfg.outDir); end

assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

%% =========================
%  ALLE RUNS LADEN UND NACH (label, source) GRUPPIEREN
%  =========================
files = dir(fullfile(cfg.runDir, '*.mat'));
assert(~isempty(files), 'Keine .mat-Dateien in %s.', cfg.runDir);

allRuns = struct([]);
for i = 1:numel(files)
    fp = fullfile(files(i).folder, files(i).name);
    L = load(fp);
    if ~isfield(L, 'data') || ~isfield(L, 'meta'), continue; end
    if ~isfield(L.meta, 'label'), continue; end
    src = '';
    if isfield(L.meta, 'source'), src = char(L.meta.source); end
    if isempty(src)
        % Fallback: Praefix der Datei interpretieren
        if startsWith(files(i).name, 'sim_'),  src = 'sim';  end
        if startsWith(files(i).name, 'run_'),  src = 'real'; end
    end
    R.file   = files(i).name;
    R.path   = fp;
    R.label  = char(L.meta.label);
    R.source = src;
    R.data   = L.data;
    R.meta   = L.meta;
    allRuns = [allRuns; R]; %#ok<AGROW>
end
fprintf('Gesamt geladene Runs: %d\n', numel(allRuns));

% Filter
if ~strcmpi(cfg.labelFilter, 'all')
    keep = ~cellfun(@isempty, regexp({allRuns.label}, cfg.labelFilter, 'once'));
    allRuns = allRuns(keep);
end

% Paare bilden
labels = unique({allRuns.label});
pairs = struct([]);
for i = 1:numel(labels)
    lab = labels{i};
    sel = allRuns(strcmp({allRuns.label}, lab));
    realIdx = find(strcmp({sel.source}, 'real'), 1, 'first');
    simIdx  = find(strcmp({sel.source}, 'sim'),  1, 'first');
    if isempty(realIdx) || isempty(simIdx)
        fprintf('  Label "%s": kein vollstaendiges Paar (real=%d, sim=%d) - skip\n', ...
            lab, ~isempty(realIdx), ~isempty(simIdx));
        continue;
    end
    P.label = lab;
    P.real  = sel(realIdx);
    P.sim   = sel(simIdx);
    pairs   = [pairs; P]; %#ok<AGROW>
end
nPairs = numel(pairs);
assert(nPairs > 0, 'Keine matching (real, sim)-Paare gefunden.');
fprintf('Gefundene Paare: %d\n', nPairs);

%% =========================
%  PRO PAAR ANALYSIEREN
%  =========================
results = struct([]);
for i = 1:nPairs
    P = pairs(i);
    fprintf('\n--- Paar %d/%d: %s ---\n', i, nPairs, P.label);

    A = preparePair(P.real, robot_rbt, cfg);   % real
    B = preparePair(P.sim,  robot_rbt, cfg);   % sim

    % Auf gemeinsame Zeitachse interpolieren (kuerzere Dauer gewinnt)
    tEnd = min(A.t(end), B.t(end));
    dtCommon = median(diff(A.t));
    if isnan(dtCommon) || dtCommon <= 0, dtCommon = 1/40; end
    tc = (0:dtCommon:tEnd)';

    Ai = resampleRun(A, tc);
    Bi = resampleRun(B, tc);

    % --- Diffs ---
    qDiff   = Ai.q   - Bi.q;       % real - sim, [deg]
    eeDiff  = Ai.ee  - Bi.ee;      % [m]
    actDiff = Ai.act - Bi.act;     % real-Action vs sim-Action, [deg/s]
    qDiffNorm  = vecnorm(qDiff,  2, 2);
    eeDiffNorm = vecnorm(eeDiff, 2, 2);
    actDiffNorm = vecnorm(actDiff, 2, 2);

    % --- Divergenzzeitpunkt t* (EE-basiert, Fallback: q-basiert) ---
    tStarEE = findCrossing(tc, eeDiffNorm, cfg.divEEThresh_m);
    tStarQ  = findCrossing(tc, max(abs(qDiff), [], 2), cfg.divQThresh_deg);

    % --- Action-Vergleich bei aehnlichen States ---
    similarPairs = compareActionsAtSimilarStates(Ai, Bi, cfg.simStateTol);

    % --- Reward / Success wenn da ---
    [rewardSum_real, rewardSum_sim] = deal(NaN);
    if ~isempty(Ai.reward), rewardSum_real = sum(Ai.reward, 'omitnan'); end
    if ~isempty(Bi.reward), rewardSum_sim  = sum(Bi.reward, 'omitnan'); end

    R.label         = P.label;
    R.tc            = tc;
    R.A             = Ai;
    R.B             = Bi;
    R.qDiff         = qDiff;
    R.eeDiff        = eeDiff;
    R.actDiff       = actDiff;
    R.qDiffNorm     = qDiffNorm;
    R.eeDiffNorm    = eeDiffNorm;
    R.actDiffNorm   = actDiffNorm;
    R.tStarEE       = tStarEE;
    R.tStarQ        = tStarQ;
    R.rmsEE         = sqrt(mean(eeDiffNorm.^2, 'omitnan'));
    R.maxEE         = max(eeDiffNorm, [], 'omitnan');
    R.rmsAction     = sqrt(mean(actDiffNorm.^2, 'omitnan'));
    R.simStatePairs = similarPairs;
    R.rewardSum_real = rewardSum_real;
    R.rewardSum_sim  = rewardSum_sim;

    fprintf('  RMS EE diff   = %.4f m   (max %.4f)\n', R.rmsEE, R.maxEE);
    fprintf('  t* (EE>%.2fm) = %s\n', cfg.divEEThresh_m, fmtTStar(R.tStarEE));
    fprintf('  t* (q>%.1f°)  = %s\n', cfg.divQThresh_deg, fmtTStar(R.tStarQ));
    if ~isempty(similarPairs.dx)
        fprintf('  Action-Diff bei aehnlichen States: median = %.3f deg/s (n=%d)\n', ...
            median(similarPairs.dActNorm, 'omitnan'), numel(similarPairs.dActNorm));
    else
        fprintf('  Action-Vergleich uebersprungen (keine obs vorhanden oder zu wenig matches)\n');
    end

    results = [results; R]; %#ok<AGROW>
end

%% =========================
%  SUMMARY-TABELLE
%  =========================
fprintf('\n=========== SIM2REAL SUMMARY ===========\n');
T = table('Size', [nPairs, 8], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'label','rmsEE_m','maxEE_m','tStarEE_s','tStarQ_s','rmsActDiff_dps','rewSum_real','rewSum_sim'});
for i = 1:nPairs
    R = results(i);
    T.label(i)            = string(R.label);
    T.rmsEE_m(i)          = R.rmsEE;
    T.maxEE_m(i)          = R.maxEE;
    T.tStarEE_s(i)        = R.tStarEE;
    T.tStarQ_s(i)         = R.tStarQ;
    T.rmsActDiff_dps(i)   = R.rmsAction;
    T.rewSum_real(i)      = R.rewardSum_real;
    T.rewSum_sim(i)       = R.rewardSum_sim;
end
disp(T);
writetable(T, fullfile(cfg.outDir, 'sim2real_summary.csv'));
fprintf('Summary -> %s\n', fullfile(cfg.outDir, 'sim2real_summary.csv'));

%% =========================
%  PLOTS
%  =========================
colors = lines(nPairs);

% -- Fig 1: EE-Bahn xz, sim vs real, ein Subplot pro Paar
nCols = min(2, nPairs); nRows = ceil(nPairs / nCols);
fig1 = figure('Name', 'EE xz: sim vs real', 'Position', [50 50 1200 800]);
for i = 1:nPairs
    R = results(i);
    subplot(nRows, nCols, i); hold on; grid on; axis equal;
    plot(R.A.ee(:,1), R.A.ee(:,3), 'b-',  'LineWidth', 1.5);
    plot(R.B.ee(:,1), R.B.ee(:,3), 'r--', 'LineWidth', 1.3);
    plot(R.A.ee(1,1), R.A.ee(1,3), 'bo', 'MarkerFaceColor', 'b');
    plot(R.B.ee(1,1), R.B.ee(1,3), 'rs', 'MarkerFaceColor', 'r');
    if ~isnan(R.tStarEE)
        idx = find(R.tc >= R.tStarEE, 1, 'first');
        plot(R.A.ee(idx,1), R.A.ee(idx,3), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    end
    xlabel('x [m]'); ylabel('z [m]');
    title(sprintf('%s   t*_{EE} = %s', R.label, fmtTStar(R.tStarEE)), ...
          'Interpreter', 'none');
    if i == 1
        legend({'real', 'sim', 'start real', 'start sim', 't*'}, 'Location', 'best');
    end
end
sgtitle('EE-Bahn in xz-Ebene: real (blau) vs. sim (rot)');
savePlot(fig1, fullfile(cfg.outDir, 'sim2real_fig1_xz'), cfg);

% -- Fig 2: Divergenz |q_real - q_sim| und |ee_real - ee_sim| ueber Zeit
fig2 = figure('Name', 'Divergence over time', 'Position', [50 50 1200 700]);
subplot(2,1,1); hold on; grid on;
for i = 1:nPairs
    plot(results(i).tc, results(i).qDiffNorm, 'Color', colors(i,:), 'LineWidth', 1.3);
end
yline(cfg.divQThresh_deg, '--k', sprintf('%.1f°', cfg.divQThresh_deg));
xlabel('t [s]'); ylabel('||q_{real} - q_{sim}|| [deg]');
title('Joint-Space Divergenz');
legend({results.label}, 'Interpreter', 'none', 'Location', 'best');

subplot(2,1,2); hold on; grid on;
for i = 1:nPairs
    plot(results(i).tc, results(i).eeDiffNorm, 'Color', colors(i,:), 'LineWidth', 1.3);
end
yline(cfg.divEEThresh_m, '--k', sprintf('%.2f m', cfg.divEEThresh_m));
xlabel('t [s]'); ylabel('||ee_{real} - ee_{sim}|| [m]');
title('Cartesian-Space Divergenz');
savePlot(fig2, fullfile(cfg.outDir, 'sim2real_fig2_divergence'), cfg);

% -- Fig 3: Pro Gelenk q_real vs q_sim ueber Zeit (1 Paar pro Figure-Block)
% Wenn viele Paare, koennte das viele Plots werden -> nur erstes Paar als Beispielplot
% Du kannst i in der Schleife aufmachen wenn du fuer alle plotten willst.
fig3 = figure('Name', 'Joint trajectories: real vs sim (first pair)', ...
              'Position', [50 50 1400 900]);
R = results(1);
for j = 1:cfg.nJoints
    subplot(4, 2, j); hold on; grid on;
    plot(R.tc, R.A.q(:, j), 'b-',  'LineWidth', 1.2);
    plot(R.tc, R.B.q(:, j), 'r--', 'LineWidth', 1.2);
    xlabel('t [s]'); ylabel(sprintf('J%d [deg]', j));
    if j == 1, legend({'real', 'sim'}, 'Location', 'best'); end
end
sgtitle(sprintf('Joint trajectories real vs sim, label=%s', R.label), ...
        'Interpreter', 'none');
savePlot(fig3, fullfile(cfg.outDir, 'sim2real_fig3_joints_pair1'), cfg);

% -- Fig 4: Action-Differenz ueber Zeit
fig4 = figure('Name', 'Action differences', 'Position', [50 50 1200 500]);
hold on; grid on;
for i = 1:nPairs
    plot(results(i).tc, results(i).actDiffNorm, ...
         'Color', colors(i,:), 'LineWidth', 1.2);
end
xlabel('t [s]'); ylabel('||a_{real}(t) - a_{sim}(t)|| [deg/s]');
title('Action-Differenz Real vs. Sim ueber Zeit');
legend({results.label}, 'Interpreter', 'none', 'Location', 'best');
savePlot(fig4, fullfile(cfg.outDir, 'sim2real_fig4_actiondiff'), cfg);

% -- Fig 5: Action-Diff bei AEHNLICHEN States (wenn obs vorhanden)
hasObs = arrayfun(@(r) ~isempty(r.simStatePairs.dx), results);
if any(hasObs)
    fig5 = figure('Name', 'Action diff at similar states');
    hold on; grid on;
    legendEntries = {};
    for i = find(hasObs(:))'
        sp = results(i).simStatePairs;
        scatter(sp.dx, sp.dActNorm, 12, colors(i,:), 'filled', 'MarkerFaceAlpha', 0.5);
        legendEntries{end+1} = results(i).label; %#ok<SAGROW>
    end
    xlabel('||obs_{real} - obs_{sim}|| (normiert)');
    ylabel('||a_{real} - a_{sim}|| [deg/s]');
    title('Policy-Konsistenz: Aktionsdifferenz bei aehnlichen Beobachtungen');
    legend(legendEntries, 'Interpreter', 'none', 'Location', 'best');
    savePlot(fig5, fullfile(cfg.outDir, 'sim2real_fig5_action_consistency'), cfg);
end

fprintf('\nAnalyse fertig. Output in: %s\n', cfg.outDir);

%% =========================
%  LOKALE FUNKTIONEN
%  =========================

function R = preparePair(rec, robot_rbt, cfg)
% Bereitet einen Run auf: einheitliches Schema {t, q, act, ee, obs, reward}.
    d = rec.data;
    t = d.t(:);

    % Auf NaN-freie Bereiche kuerzen
    if isfield(d, 'q_measured')
        valid = ~any(isnan(d.q_measured), 2) & ~isnan(t);
    else
        valid = ~isnan(t);
    end
    t = t(valid);

    q = []; act = []; ee = []; obs = []; reward = [];
    if isfield(d, 'q_measured'), q = d.q_measured(valid, :); end
    if isfield(d, 'dq_cmd'),     act = d.dq_cmd(valid, :);   end

    if isfield(d, 'ee_measured') && ~isempty(d.ee_measured)
        ee = d.ee_measured(valid, :);
    elseif ~isempty(q)
        ee = zeros(numel(t), 3);
        for k = 1:numel(t)
            ee(k, :) = fkEE(robot_rbt, q(k, :), cfg.eeBodyName, cfg.toolOffset);
        end
    end

    if isfield(d, 'obs') && ~isempty(d.obs)
        obs = d.obs(valid, :);
    end
    if isfield(d, 'reward') && ~isempty(d.reward)
        reward = d.reward(valid, :);
    end

    R.t = t; R.q = q; R.act = act; R.ee = ee; R.obs = obs; R.reward = reward;
    R.label = rec.label; R.source = rec.source;
end

function Ri = resampleRun(R, tc)
% Linear interpoliert alle vorhandenen Felder auf gemeinsame Zeitachse tc.
    Ri.t = tc;
    Ri.q  = safeInterp(R.t, R.q,  tc);
    Ri.act= safeInterp(R.t, R.act,tc);
    Ri.ee = safeInterp(R.t, R.ee, tc);
    Ri.obs= safeInterp(R.t, R.obs,tc);
    Ri.reward = safeInterp(R.t, R.reward, tc);
end

function Y = safeInterp(t, X, tc)
    if isempty(X) || isempty(t)
        Y = []; return;
    end
    Y = zeros(numel(tc), size(X, 2));
    for c = 1:size(X, 2)
        Y(:, c) = interp1(t, X(:, c), tc, 'linear', 'extrap');
    end
end

function tStar = findCrossing(t, x, thr)
    idx = find(x >= thr, 1, 'first');
    if isempty(idx)
        tStar = NaN;
    else
        tStar = t(idx);
    end
end

function s = fmtTStar(tStar)
    if isnan(tStar)
        s = 'never';
    else
        s = sprintf('%.2f s', tStar);
    end
end

function out = compareActionsAtSimilarStates(Ai, Bi, tol)
% Sucht Zeitpunkte, an denen obs_real(t) ~ obs_sim(t) (innerhalb tol nach
% Normierung), und vergleicht dort die Aktionen.
    out.dx = []; out.dActNorm = [];
    if isempty(Ai.obs) || isempty(Bi.obs) || isempty(Ai.act) || isempty(Bi.act)
        return;
    end
    % Per-Spalte normieren mit Range aus real-Daten (robust)
    rng = max(Ai.obs, [], 1) - min(Ai.obs, [], 1);
    rng(rng < 1e-9) = 1;
    obsA = Ai.obs ./ rng;
    obsB = Bi.obs ./ rng;
    dx = vecnorm(obsA - obsB, 2, 2);
    da = vecnorm(Ai.act - Bi.act, 2, 2);
    sel = dx < tol;
    out.dx       = dx(sel);
    out.dActNorm = da(sel);
end

function p = fkEE(robot_rbt, q_deg, eeBodyName, toolOffset)
    T = getTransform(robot_rbt, deg2rad(q_deg(:).'), char(eeBodyName));
    if nargin >= 4 && ~isempty(toolOffset) && any(toolOffset ~= 0)
        p = (T(1:3,4) + T(1:3,1:3)*toolOffset(:)).';
    else
        p = T(1:3,4).';
    end
end

function savePlot(figHandle, basepath, cfg)
    if ~cfg.savePlots, return; end
    for k = 1:numel(cfg.plotFormats)
        fmt = cfg.plotFormats{k};
        try
            exportgraphics(figHandle, [basepath '.' fmt], 'Resolution', 200);
        catch
            saveas(figHandle, [basepath '.' fmt]);
        end
    end
end

%% analyze_tracking_error.m
% Offline-Auswertung aller Playback-Runs aus runs/.
% Berechnet pro Run:
%   - Gelenk-Tracking-Fehler q_measured - q_integrated(dq_cmd)
%   - Kartesischen EE-Fehler real-FK vs integriert-FK
%   - Korrelation |dq_cmd| ~ |Trackingfehler| pro Gelenk
%   - Max/RMS Zusammenfassung
%
% Erzeugt Plots (Figures + gespeicherte PNG/PDF) und eine
% Summary-Tabelle, die direkt in die Bachelorarbeit wandern kann.
%
% Abhaengigkeiten:
%   - URDF-Datei (gleich wie im Playback/FK-Skript)
%   - runs/ Ordner mit run_XXX_*.mat Dateien aus run_logger
%
% Input-Erwartung pro Run-Datei:
%   data.t          Nx1  [s]
%   data.q_measured Nx7  [deg]
%   data.dq_cmd     Nx7  [deg/s]
%   data.ee_measured Nx3 [m] (optional; wird sonst via FK nachberechnet)
%   meta.label, meta.startpose, meta.timeScale, meta.startposeName, ...

clear; clc; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg.runDir     = sk_path('data', 'hardware', 'playback_runs');   % Playback-Laeufe 001-006
cfg.outDir     = sk_path('data', 'hardware', 'analysis_output');
cfg.urdfFile   = sk_path('robot', 'GEN3-7DOF-VISION_ARM_URDF_V12.urdf');
cfg.eeBodyName = 'end_effector_link';
cfg.toolOffset = [0; 0; 0];
cfg.nJoints    = 7;

% Welche Runs analysieren?
% Es werden bewusst NUR diese sechs Dateien aus dem Screenshot verwendet.
% Die Reihenfolge legt gleichzeitig die spaetere Plot-Anordnung fest:
%   obere Zeile  = singular
%   untere Zeile = non-singular
%   Spalten      = baseline, slow, fast
cfg.runFiles = { ...
    'run_001_kin_baseline_training_singular.mat', ...
    'run_002_kin_slow_training_singular.mat', ...
    'run_003_kin_fast_training_singular.mat', ...
    'run_004_kin_baseline_training_non-singular.mat', ...
    'run_005_kin_slow_training_non-singular.mat', ...
    'run_006_kin_fast_training_non-singular.mat' ...
};
cfg.savePlots  = true;
cfg.plotFormats = {'png', 'pdf'};    % wird pro Figure gespeichert

%% =========================
%  VORBEREITUNG
%  =========================
assert(isfolder(cfg.runDir), 'Run-Ordner fehlt: %s', cfg.runDir);
if ~isfolder(cfg.outDir)
    mkdir(cfg.outDir);
end

% Nur explizit freigegebene Dateien verwenden und exakt in dieser Reihenfolge sortieren.
allFiles = dir(fullfile(cfg.runDir, 'run_*.mat'));
assert(~isempty(allFiles), 'Keine Run-Dateien in %s gefunden.', cfg.runDir);

files = struct('name', {}, 'folder', {}, 'date', {}, 'bytes', {}, ...
               'isdir', {}, 'datenum', {});
for k = 1:numel(cfg.runFiles)
    idx = find(strcmp({allFiles.name}, cfg.runFiles{k}), 1);
    if isempty(idx)
        warning('Gewuenschte Datei nicht gefunden und wird uebersprungen: %s', cfg.runFiles{k});
    else
        files(end+1) = allFiles(idx); %#ok<SAGROW>
    end
end
assert(~isempty(files), 'Keine der gewuenschten Run-Dateien wurde in %s gefunden.', cfg.runDir);

fprintf('Analysierte Runs aus Screenshot: %d von %d\n', numel(files), numel(cfg.runFiles));

% URDF laden
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

%% =========================
%  ALLE RUNS LADEN UND METRIKEN BERECHNEN
%  =========================
runs = struct([]);
for i = 1:numel(files)
    fp = fullfile(files(i).folder, files(i).name);
    fprintf('\n[%d/%d] Lade %s\n', i, numel(files), files(i).name);

    L = load(fp);
    if ~isfield(L, 'data') || ~isfield(L, 'meta')
        warning('  Kein data/meta struct, ueberspringe.');
        continue;
    end
    data = L.data; meta = L.meta;

    % --- Basisgroessen ---
    if ~isfield(data, 't') || ~isfield(data, 'q_measured') || ~isfield(data, 'dq_cmd')
        warning('  Pflichtfelder fehlen, ueberspringe.');
        continue;
    end
    t = data.t(:);
    q_meas_deg  = data.q_measured;
    dq_cmd_deg  = data.dq_cmd;

    % Auf NaN-freie Bereiche beschneiden (runPlayback preallocs mit NaN)
    valid = ~any(isnan(q_meas_deg), 2) & ~isnan(t);
    t          = t(valid);
    q_meas_deg = q_meas_deg(valid, :);
    dq_cmd_deg = dq_cmd_deg(valid, :);
    if numel(t) < 3
        warning('  Zu wenige gueltige Samples, ueberspringe.');
        continue;
    end

    % --- q_integrated = forward-Euler ueber dq_cmd, Start = q_measured(1) ---
    % Damit ist der Anfangs-Offset 0 und wir messen nur den kumulierten
    % Tracking-Fehler, nicht eine beliebige Startdifferenz.
    q_int_deg = zeros(size(q_meas_deg));
    q_int_deg(1, :) = q_meas_deg(1, :);
    for k = 2:numel(t)
        dt_k = t(k) - t(k-1);
        q_int_deg(k, :) = q_int_deg(k-1, :) + dq_cmd_deg(k-1, :) * dt_k;
    end

    % --- Tracking-Fehler pro Gelenk ---
    jointErr_deg = q_meas_deg - q_int_deg;

    % --- Kartesischer Vergleich: FK auf q_meas vs q_int ---
    ee_meas = zeros(numel(t), 3);
    ee_int  = zeros(numel(t), 3);
    for k = 1:numel(t)
        ee_meas(k, :) = fkEE(robot_rbt, q_meas_deg(k, :), ...
                             cfg.eeBodyName, cfg.toolOffset);
        ee_int(k, :)  = fkEE(robot_rbt, q_int_deg(k, :), ...
                             cfg.eeBodyName, cfg.toolOffset);
    end
    if isfield(data, 'ee_measured') && all(size(data.ee_measured) == size(ee_meas))
        % Falls online geloggt, als sanity check verwenden
        delta = max(abs(data.ee_measured(valid, :) - ee_meas), [], 'all');
        if delta > 1e-3
            warning('  EE-Online vs EE-Offline weicht um %.4f m ab.', delta);
        end
    end
    ee_err = vecnorm(ee_meas - ee_int, 2, 2);

    % --- Korrelation |dq_cmd| vs |jointErr| pro Gelenk ---
    corrJoint = zeros(1, cfg.nJoints);
    for j = 1:cfg.nJoints
        a = abs(dq_cmd_deg(:, j));
        b = abs(jointErr_deg(:, j));
        if std(a) > 1e-6 && std(b) > 1e-6
            c = corrcoef(a, b);
            corrJoint(j) = c(1, 2);
        else
            corrJoint(j) = NaN;
        end
    end

    % --- Summary-Metriken ---
    rmsJoint = sqrt(mean(jointErr_deg.^2, 1));
    maxJoint = max(abs(jointErr_deg), [], 1);
    rmsEE    = sqrt(mean(ee_err.^2));
    maxEE    = max(ee_err);

    % --- In Struct sammeln ---
    R.file       = files(i).name;
    [R.poseGroup, R.speedGroup, R.layoutRow, R.layoutCol] = classifyRunForLayout(files(i).name);
    R.label      = getFieldOr(meta, 'label', sprintf('run%d', i));
    R.poseName   = getFieldOr(meta, 'startposeName', '');
    R.timeScale  = getFieldOr(meta, 'timeScale', NaN);
    R.t          = t;
    R.q_meas     = q_meas_deg;
    R.q_int      = q_int_deg;
    R.dq_cmd     = dq_cmd_deg;
    R.jointErr   = jointErr_deg;
    R.ee_meas    = ee_meas;
    R.ee_int     = ee_int;
    R.ee_err     = ee_err;
    R.rmsJoint   = rmsJoint;
    R.maxJoint   = maxJoint;
    R.rmsEE      = rmsEE;
    R.maxEE      = maxEE;
    R.corrJoint  = corrJoint;
    runs = [runs; R]; %#ok<AGROW>

    fprintf('  label=%s, pose=%s, ts=%.2f\n', R.label, R.poseName, R.timeScale);
    fprintf('  RMS EE = %.4f m, Max EE = %.4f m\n', rmsEE, maxEE);
end

assert(~isempty(runs), 'Keine Runs erfolgreich analysiert.');
nRuns = numel(runs);

%% =========================
%  SUMMARY-TABELLE
%  =========================
fprintf('\n=========== SUMMARY ===========\n');
varNames = {'label', 'pose', 'timeScale', 'RMS_EE_m', 'Max_EE_m', ...
            'RMS_J1', 'RMS_J2', 'RMS_J3', 'RMS_J4', 'RMS_J5', 'RMS_J6', 'RMS_J7'};
T = table('Size', [nRuns, numel(varNames)], ...
    'VariableTypes', [{'string', 'string'}, repmat({'double'}, 1, numel(varNames)-2)], ...
    'VariableNames', varNames);
for i = 1:nRuns
    T.label(i)     = string(runs(i).label);
    T.pose(i)      = string(runs(i).poseName);
    T.timeScale(i) = runs(i).timeScale;
    T.RMS_EE_m(i)  = runs(i).rmsEE;
    T.Max_EE_m(i)  = runs(i).maxEE;
    for j = 1:cfg.nJoints
        T.(sprintf('RMS_J%d', j))(i) = runs(i).rmsJoint(j);
    end
end
disp(T);
writetable(T, fullfile(cfg.outDir, 'summary.csv'));
fprintf('Summary gespeichert: %s\n', fullfile(cfg.outDir, 'summary.csv'));

%% =========================
%  FARBEN / LEGENDE
%  =========================
colors = lines(nRuns);
legendLabels = arrayfun(@(r) sprintf('%s | %s | ts=%.2f', ...
                        r.poseGroup, r.speedGroup, r.timeScale), ...
                        runs, 'UniformOutput', false);

%% =========================
%  FIGURE 1: Gelenkfehler ueber Zeit, 7 Subplots
%  =========================
fig1 = figure('Name', 'Joint Tracking Error', 'Position', [50 50 1400 900]);
for j = 1:cfg.nJoints
    subplot(4, 2, j); hold on; grid on;
    for i = 1:nRuns
        plot(runs(i).t, runs(i).jointErr(:, j), ...
             'Color', colors(i, :), 'LineWidth', 1.1);
    end
    xlabel('t [s]'); ylabel(sprintf('J%d err [deg]', j));
    title(sprintf('Gelenk %d: q_{meas} - q_{int}', j));
    if j == 1
        legend(legendLabels, 'Location', 'best', 'Interpreter', 'none');
    end
end
sgtitle('Tracking-Fehler pro Gelenk (real - integriert)');
savePlot(fig1, fullfile(cfg.outDir, 'fig1_joint_error'), cfg);

%% =========================
%  FIGURE 2: Kartesischer EE-Fehler ueber Zeit
%  =========================
fig2 = figure('Name', 'Cartesian EE Error'); hold on; grid on;
for i = 1:nRuns
    plot(runs(i).t, runs(i).ee_err, ...
         'Color', colors(i, :), 'LineWidth', 1.4);
end
xlabel('t [s]');
ylabel('||p_{meas} - p_{int}|| [m]');
title('Kartesischer EE-Fehler (real vs. integriert)');
legend(legendLabels, 'Location', 'best', 'Interpreter', 'none');
savePlot(fig2, fullfile(cfg.outDir, 'fig2_ee_error'), cfg);

%% =========================
%  FIGURE 3a: xy-Bahn pro Run, festes 2x3-Gitter
%  =========================
fig3a = figure('Name', 'xy Trajectories', 'Position', [50 50 1500 850]);
plotTrajectoryGrid(runs, fig3a, 1, 2, ...
    'x [m]', 'y [m]', ...
    'EE-Bahn in xy-Ebene: real/FK aus Messwerten vs. FK-Prognose aus dq_{cmd}', ...
    fullfile(cfg.outDir, 'fig3a_xy_trajectories'), cfg);

%% =========================
%  FIGURE 3b: xz-Bahn pro Run, festes 2x3-Gitter
%  =========================
fig3b = figure('Name', 'xz Trajectories', 'Position', [50 50 1500 850]);
plotTrajectoryGrid(runs, fig3b, 1, 3, ...
    'x [m]', 'z [m]', ...
    'EE-Bahn in xz-Ebene: real/FK aus Messwerten vs. FK-Prognose aus dq_{cmd}', ...
    fullfile(cfg.outDir, 'fig3b_xz_trajectories'), cfg);

%% =========================
%  FIGURE 4: Korrelation |dq_cmd| vs |Trackingfehler|
%  =========================
fig4 = figure('Name', 'Corr dq vs err', 'Position', [50 50 1400 900]);
for j = 1:cfg.nJoints
    subplot(4, 2, j); hold on; grid on;
    for i = 1:nRuns
        a = abs(runs(i).dq_cmd(:, j));
        b = abs(runs(i).jointErr(:, j));
        scatter(a, b, 8, colors(i, :), 'filled', 'MarkerFaceAlpha', 0.4);
    end
    xlabel(sprintf('|dq_{cmd,%d}| [deg/s]', j));
    ylabel(sprintf('|err_{%d}| [deg]', j));
    corrTxt = arrayfun(@(r) sprintf('%.2f', r.corrJoint(j)), runs, ...
                       'UniformOutput', false);
    title(sprintf('J%d  corr = [%s]', j, strjoin(corrTxt, ', ')));
end
sgtitle('Korrelation |dq_{cmd}| vs. |Tracking-Fehler| pro Gelenk');
savePlot(fig4, fullfile(cfg.outDir, 'fig4_corr_dq_err'), cfg);

%% =========================
%  FIGURE 5: RMS-Gelenkfehler als Balken
%  =========================
fig5 = figure('Name', 'RMS Joint Errors'); 
rmsMat = zeros(nRuns, cfg.nJoints);
for i = 1:nRuns
    rmsMat(i, :) = runs(i).rmsJoint;
end
bar(rmsMat');
grid on;
xlabel('Gelenk'); ylabel('RMS Fehler [deg]');
xticks(1:cfg.nJoints); xticklabels(compose('J%d', 1:cfg.nJoints));
title('RMS Gelenk-Tracking-Fehler pro Run');
legend(legendLabels, 'Location', 'bestoutside', 'Interpreter', 'none');
savePlot(fig5, fullfile(cfg.outDir, 'fig5_rms_bars'), cfg);

fprintf('\nAnalyse fertig. Plots & Tabelle in: %s\n', cfg.outDir);

%% =========================
%  LOKALE FUNKTIONEN
%  =========================

function [poseGroup, speedGroup, row, col] = classifyRunForLayout(fileName)
    % Ordnet die sechs freigegebenen Dateien fest in ein 2x3-Layout ein.
    % row = 1: singular, row = 2: non-singular
    % col = 1: baseline, col = 2: slow, col = 3: fast
    if contains(fileName, 'training_non-singular')
        poseGroup = 'Non-singular';
        row = 2;
    elseif contains(fileName, 'training_singular')
        poseGroup = 'Singular';
        row = 1;
    else
        poseGroup = 'Unbekannt';
        row = NaN;
    end

    if contains(fileName, '_baseline_')
        speedGroup = 'Baseline';
        col = 1;
    elseif contains(fileName, '_slow_')
        speedGroup = 'Slow';
        col = 2;
    elseif contains(fileName, '_fast_')
        speedGroup = 'Fast';
        col = 3;
    else
        speedGroup = 'Unbekannt';
        col = NaN;
    end
end

function plotTrajectoryGrid(runs, figHandle, dimA, dimB, xLabelTxt, yLabelTxt, mainTitle, basepath, cfg)
    % Zeichnet pro Run einen festen Platz im 2x3-Gitter:
    % oben: Singular, unten: Non-singular; Spalten: Baseline, Slow, Fast.
    figure(figHandle);

    rowNames = {'Singular', 'Non-singular'};
    colNames = {'Baseline', 'Slow', 'Fast'};

    for row = 1:2
        for col = 1:3
            ax = subplot(2, 3, (row-1)*3 + col); %#ok<LAXES>
            hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

            idx = find([runs.layoutRow] == row & [runs.layoutCol] == col, 1);
            if isempty(idx)
                title(ax, sprintf('%s | %s\nDatei fehlt', rowNames{row}, colNames{col}), ...
                    'Interpreter', 'none');
                xlabel(ax, xLabelTxt); ylabel(ax, yLabelTxt);
                continue;
            end

            r = runs(idx);

            % Real/FK aus gemessenen Gelenkwinkeln
            plot(ax, r.ee_meas(:, dimA), r.ee_meas(:, dimB), ...
                'b-', 'LineWidth', 1.6, 'DisplayName', 'Real/FK aus q_{meas}');

            % Wesentlicher Bestandteil: FK-Prognose aus integrierter dq_cmd
            plot(ax, r.ee_int(:, dimA), r.ee_int(:, dimB), ...
                'r--', 'LineWidth', 1.5, 'DisplayName', 'FK-Prognose aus dq_{cmd}');

            % Start- und Endpunkte markieren
            plot(ax, r.ee_meas(1, dimA), r.ee_meas(1, dimB), ...
                'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start real');
            plot(ax, r.ee_int(1, dimA), r.ee_int(1, dimB), ...
                'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'Start Prognose');
            plot(ax, r.ee_meas(end, dimA), r.ee_meas(end, dimB), ...
                'bo', 'MarkerFaceColor', 'b', 'DisplayName', 'Ende real');
            plot(ax, r.ee_int(end, dimA), r.ee_int(end, dimB), ...
                'rs', 'MarkerFaceColor', 'r', 'DisplayName', 'Ende Prognose');

            xlabel(ax, xLabelTxt); ylabel(ax, yLabelTxt);
            title(ax, sprintf('%s | %s\n%s', rowNames{row}, colNames{col}, r.file), ...
                'Interpreter', 'none');

            if row == 1 && col == 1
                legend(ax, 'Location', 'best', 'Interpreter', 'tex');
            end
        end
    end

    sgtitle(mainTitle, 'Interpreter', 'tex');
    savePlot(figHandle, basepath, cfg);
end

function p = fkEE(robot_rbt, q_deg, eeBodyName, toolOffset)
    T = getTransform(robot_rbt, deg2rad(q_deg(:).'), char(eeBodyName));
    if nargin >= 4 && ~isempty(toolOffset) && any(toolOffset ~= 0)
        p = (T(1:3, 4) + T(1:3, 1:3) * toolOffset(:)).';
    else
        p = T(1:3, 4).';
    end
end

function v = getFieldOr(s, f, default)
    if isfield(s, f) && ~isempty(s.(f))
        v = s.(f);
    else
        v = default;
    end
end

function savePlot(figHandle, basepath, cfg)
    if ~cfg.savePlots
        return;
    end
    for k = 1:numel(cfg.plotFormats)
        fmt = cfg.plotFormats{k};
        try
            exportgraphics(figHandle, [basepath '.' fmt], 'Resolution', 200);
        catch
            saveas(figHandle, [basepath '.' fmt]);
        end
    end
end

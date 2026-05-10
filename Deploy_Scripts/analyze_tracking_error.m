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
cfg.runDir     = 'runs';
cfg.outDir     = 'analysis_output';
cfg.urdfFile   = '../GEN3-7DOF-VISION_ARM_URDF_V12.urdf';
cfg.eeBodyName = 'end_effector_link';
cfg.toolOffset = [0; 0; 0];
cfg.nJoints    = 7;

% Welche Runs analysieren? Regex auf Dateinamen oder 'all'.
cfg.runFilter  = 'all';              % z.B. 'kin_baseline' oder 'all'
cfg.savePlots  = true;
cfg.plotFormats = {'png', 'pdf'};    % wird pro Figure gespeichert

%% =========================
%  VORBEREITUNG
%  =========================
assert(isfolder(cfg.runDir), 'Run-Ordner fehlt: %s', cfg.runDir);
if ~isfolder(cfg.outDir)
    mkdir(cfg.outDir);
end

files = dir(fullfile(cfg.runDir, 'run_*.mat'));
assert(~isempty(files), 'Keine Run-Dateien in %s gefunden.', cfg.runDir);

if ~strcmpi(cfg.runFilter, 'all')
    keep = ~cellfun(@isempty, regexp({files.name}, cfg.runFilter, 'once'));
    files = files(keep);
    assert(~isempty(files), 'Kein Run passt zum Filter "%s".', cfg.runFilter);
end

fprintf('Gefundene Runs: %d\n', numel(files));

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
legendLabels = arrayfun(@(r) sprintf('%s (ts=%.2f)', r.label, r.timeScale), ...
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
%  FIGURE 3: xz-Bahn pro Run, Subplot-Gitter
%  =========================
nCols = min(2, nRuns);
nRows = ceil(nRuns / nCols);
fig3 = figure('Name', 'xz Trajectories', 'Position', [50 50 1200 800]);
for i = 1:nRuns
    subplot(nRows, nCols, i); hold on; grid on; axis equal;
    plot(runs(i).ee_meas(:, 1), runs(i).ee_meas(:, 3), 'b-', 'LineWidth', 1.5);
    plot(runs(i).ee_int(:, 1),  runs(i).ee_int(:, 3),  'r--', 'LineWidth', 1.3);
    plot(runs(i).ee_meas(1, 1), runs(i).ee_meas(1, 3), 'go', 'MarkerFaceColor', 'g');
    plot(runs(i).ee_int(1, 1),  runs(i).ee_int(1, 3),  'ks', 'MarkerFaceColor', 'k');
    xlabel('x [m]'); ylabel('z [m]');
    title(sprintf('%s (ts=%.2f)', runs(i).label, runs(i).timeScale), ...
          'Interpreter', 'none');
    if i == 1
        legend({'real (FK auf q_{meas})', 'integriert (FK auf q_{int})', ...
                'Start real', 'Start int'}, 'Location', 'best');
    end
end
sgtitle('EE-Bahn in xz-Ebene: real vs. integriert');
savePlot(fig3, fullfile(cfg.outDir, 'fig3_xz_trajectories'), cfg);

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

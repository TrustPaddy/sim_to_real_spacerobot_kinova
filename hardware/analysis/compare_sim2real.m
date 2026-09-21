%% compare_sim2real.m
% Symmetrischer Sim-to-Real-Vergleich fuer SpaceKinova-Runs.
%
% Idee:
%   - Sim- UND Real-Runs liegen im selben run_logger-Schema (data/meta) im
%     selben Ordner. Quelle wird ueber meta.source ('sim' | 'real') getrennt.
%   - Es wird NUR das gemeinsame, physikalisch vergleichbare KPI-Subset
%     berechnet (EE-Positionsfehler, dq_cmd, Completion). Basis-Stoerung,
%     Drehmoment, Reward existieren real nicht und werden NICHT verglichen.
%   - Beide Quellen laufen durch IDENTISCHEN KPI-Code (computeCommonKPIs).
%   - Trajektorien werden auf normierte PHASE [0,1] resampled, nicht auf Zeit.
%     Dadurch ist der Positionspfad-/Positionsfehler-Vergleich auch dann
%     gueltig, wenn Sim (T=8.5 s) und Real (maxDuration=17 s) den Halbkreis
%     unterschiedlich schnell fahren. ACHTUNG: Geschwindigkeits-/Effort-KPIs
%     (cmd_rate) sind erst vergleichbar, wenn Dauer UND die 0.35-Derating im
%     Deploy in der Sim gespiegelt sind.
%
% TASK-FILTER:
%   Im runs/-Ordner liegen mehrere Tasks gemischt:
%     - sim_circle_*      (Sim,  Halbkreis)        -> behalten
%     - agent_train_*     (Real, Halbkreis)        -> behalten
%     - agent_p2p_*       (Real, Punkt-zu-Punkt)   -> AUSSCHLIESSEN
%   Der p2p-Task hat eine andere Referenz/Dauer und darf NICHT in den
%   Halbkreis-Vergleich gemittelt werden. Gefiltert wird ueber den
%   Dateinamen (cfg.excludeLabel / cfg.includeLabel).
%
% Aufruf: einfach als Skript ausfuehren, cfg unten anpassen.

clc; clear; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();
cfg.runDir       = sk_path("data", "hardware", "runs");   % Ordner mit run_*.mat
cfg.pattern      = "run_*.mat";

% --- Task-Filter ---
cfg.excludeLabel = "p2p";         % Dateien, deren Name das enthaelt, werden uebersprungen ("" = nichts)
cfg.includeLabel = "";            % falls gesetzt: NUR Dateien, deren Name das enthaelt ("" = alle)

% --- EE-Quelle / Resampling / Einheiten ---
cfg.eeSourceReal = "fk";          % "fk" (Modell-EE) | "kortex" (HW-Wahrheit)
cfg.nGrid        = 200;           % Aufloesung des Phasen-Gitters [0,1]
cfg.dqRealUnits  = "deg";         % Fallback-Einheit dq_cmd real (wenn meta.units fehlt)
cfg.dqSimUnits   = "deg";         % Fallback-Einheit dq_cmd sim
cfg.saveSummary  = true;          % Summary als .mat + .csv ablegen

%% =========================
%  1) RUNS LADEN + FILTERN + NACH QUELLE TRENNEN
%  =========================
files = dir(fullfile(cfg.runDir, cfg.pattern));
assert(~isempty(files), "Keine Runs gefunden in %s/%s", cfg.runDir, cfg.pattern);

runsCell = {};
skipped  = strings(0,1);
for i = 1:numel(files)
    nm = string(files(i).name);

    % --- Task-Filter (ueber Dateiname) ---
    if strlength(cfg.excludeLabel) > 0 && contains(lower(nm), lower(cfg.excludeLabel))
        skipped(end+1) = nm; %#ok<AGROW>
        continue;
    end
    if strlength(cfg.includeLabel) > 0 && ~contains(lower(nm), lower(cfg.includeLabel))
        skipped(end+1) = nm; %#ok<AGROW>
        continue;
    end

    fpath = fullfile(files(i).folder, files(i).name);
    try
        runsCell{end+1} = loadRun(fpath, cfg); %#ok<AGROW>
    catch ME
        fprintf("  [SKIP-FEHLER] %s: %s\n", nm, ME.message);
    end
end

% Cell -> Struct-Array (alle Runs haben identische Felder aus loadRun).
if isempty(runsCell)
    runs = struct([]);
else
    runs = [runsCell{:}];
end
assert(~isempty(runs), "Nach dem Filtern sind keine Runs uebrig.");

if ~isempty(skipped)
    fprintf("Ausgefiltert (%d): %s\n", numel(skipped), strjoin(skipped, ", "));
end

srcs    = string({runs.source});
idxSim  = find(srcs == "sim");
idxReal = find(srcs == "real");

fprintf("Geladen: %d Runs (%d sim, %d real)\n", ...
        numel(runs), numel(idxSim), numel(idxReal));
fprintf("  Sim-Labels : %s\n",  strjoin(unique(string({runs(idxSim).label})),  ", "));
fprintf("  Real-Labels: %s\n",  strjoin(unique(string({runs(idxReal).label})), ", "));
if isempty(idxSim)
    warning("Keine Sim-Runs -- nur reale Statistik.");
end
if isempty(idxReal)
    warning("Keine Real-Runs -- nur Sim-Statistik.");
end

%% =========================
%  2) PRO-RUN KPIs (gemeinsames Subset)
%  =========================
kpiAll = arrayfun(@computeCommonKPIs, runs);

aggSim  = aggregateKPIs(kpiAll(idxSim));
aggReal = aggregateKPIs(kpiAll(idxReal));

printComparisonTable(aggSim, aggReal);

%% =========================
%  3) PHASEN-RESAMPLING (fuer Baender / Pfad-Overlay)
%  =========================
sgrid = linspace(0, 1, cfg.nGrid).';

[eeX_sim,  eeZ_sim,  ep_sim ] = resampleGroup(runs(idxSim),  sgrid);
[eeX_real, eeZ_real, ep_real] = resampleGroup(runs(idxReal), sgrid);

% Referenz auf Phase: in Sim und Real geometrisch identisch (omega*t: 0->pi).
[refX, refZ] = resampleRef(runs, sgrid);

%% =========================
%  4) PLOTS
%  =========================

% (a) XZ-Pfad-Overlay (Mittelwerte) + geteilte Referenz
figure('Name', 'sim2real: EE-Pfad (XZ)');
plot(refX, refZ, 'k:', 'LineWidth', 1.4); hold on;
hLeg = {'Referenz'};
if ~isempty(idxSim)
    plot(mean(eeX_sim,2,'omitnan'), mean(eeZ_sim,2,'omitnan'), 'b-', 'LineWidth', 1.6);
    hLeg{end+1} = sprintf('Sim (n=%d)', numel(idxSim));
end
if ~isempty(idxReal)
    plot(mean(eeX_real,2,'omitnan'), mean(eeZ_real,2,'omitnan'), 'r--', 'LineWidth', 1.6);
    hLeg{end+1} = sprintf('Real (n=%d)', numel(idxReal));
end
grid on; axis equal; xlabel('x [m]'); ylabel('z [m]');
legend(hLeg, 'Location', 'best');
title('Mittlerer EE-Pfad: Sim vs. Real (phasen-aligned)');

% (b) ||ep|| ueber Phase, mit +/-1 Std-Band
figure('Name', 'sim2real: Positionsfehler ueber Phase');
hold on; hLeg = {};
if ~isempty(idxSim)
    plotBand(sgrid, ep_sim, [0 0 1]);  hLeg{end+1} = 'Sim \mu\pm\sigma';
end
if ~isempty(idxReal)
    plotBand(sgrid, ep_real, [1 0 0]); hLeg{end+1} = 'Real \mu\pm\sigma';
end
grid on; xlabel('Trajektorien-Phase [-]'); ylabel('||e_p|| [m]');
legend(hLeg, 'Location', 'best');
title('EE-Positionsfehler ueber normierte Phase');

% (c) Positions-KPIs als Balken (phasen-vergleichbar)
plotKpiBars(aggSim, aggReal, ...
    {'rmse_ep','mae_ep','max_ep','final_ep'}, ...
    {'RMSE','MAE','Max','Endfehler'}, ...
    'Positions-KPIs [m] (phasen-vergleichbar)');

% (d) Effort-KPI (NUR vergleichbar bei gleicher Dauer + gespiegeltem Derating!)
plotKpiBars(aggSim, aggReal, {'cmd_rate'}, {'||\Deltadq_{cmd}||/dt'}, ...
    'Kommando-Rate [rad/s^2]  (nur bei gleicher Dauer/Derating vergleichbar)');

%% =========================
%  5) SUMMARY SPEICHERN
%  =========================
if cfg.saveSummary
    summary = struct('cfg', cfg, 'aggSim', aggSim, 'aggReal', aggReal, ...
                     'perRun', kpiAll, 'src', {srcs});
    stamp = string(datetime('now'), 'yyyyMMdd_HHmmss');
    save(fullfile(cfg.runDir, sprintf("sim2real_summary_%s.mat", stamp)), '-struct', 'summary');
    writeSummaryCsv(fullfile(cfg.runDir, sprintf("sim2real_kpis_%s.csv", stamp)), runs, kpiAll);
    fprintf("\nSummary gespeichert in %s/\n", cfg.runDir);
end

%% =====================================================================
%  LOKALE FUNKTIONEN
%  =====================================================================

function r = loadRun(fpath, cfg)
% Laedt einen run_*.mat und normalisiert ihn auf ein kanonisches In-Memory-
% Schema (SI-Einheiten): t [s], ee [m], ee_ref [m], ep [m], ep_norm [m],
% dq_cmd [rad/s], plus source/label/Tref.
    S = load(fpath);
    if isfield(S,'data') && isfield(S,'meta')
        data = S.data; meta = S.meta;
    elseif isfield(S,'run') && isfield(S.run,'data')
        data = S.run.data; meta = S.run.meta;
    else
        error("Unbekanntes Run-Format (kein data/meta).");
    end

    source = lower(string(meta.source));

    % --- Zeit ---
    if     isfield(data,'t'),     t = data.t(:);
    elseif isfield(data,'t_ref'), t = data.t_ref(:);
    else,  error("Keine Zeitbasis (t / t_ref) im Run."); end

    % --- EE-Position ---
    if source == "real" && cfg.eeSourceReal == "kortex" ...
            && isfield(data,'ee_kortex') && ~all(isnan(data.ee_kortex(:)))
        ee = data.ee_kortex;
    else
        ee = data.ee_measured;          % Modell-/FK-EE (sim: p_EE)
    end

    ee_ref = data.ee_ref;

    if isfield(data,'ep'), ep = data.ep; else, ep = ee - ee_ref; end
    if isfield(data,'ep_norm'), ep_norm = data.ep_norm(:); else, ep_norm = vecnorm(ep,2,2); end

    % --- dq_cmd in rad/s normalisieren ---
    dq = [];
    if isfield(data,'dq_cmd')
        dq = data.dq_cmd;
        units = unitOf(meta, 'dq_cmd', ternary(source=="real", cfg.dqRealUnits, cfg.dqSimUnits));
        if units == "deg", dq = deg2rad(dq); end
    end

    % --- Intendierte Dauer fuer Phasen-Normierung ---
    if isfield(meta,'maxDuration'), Tref = meta.maxDuration;
    elseif numel(t) >= 2,           Tref = t(end) - t(1);
    else,                           Tref = 1; end

    if isfield(meta,'label'), label = string(meta.label); else, label = "?"; end

    r = struct('source', source, 'label', label, 't', t, 'Tref', Tref, ...
               'ee', ee, 'ee_ref', ee_ref, 'ep', ep, 'ep_norm', ep_norm, 'dq_cmd', dq);
end

function kpi = computeCommonKPIs(r)
% Gemeinsames KPI-Subset -- identisch fuer sim und real.
    ep = r.ep_norm; t = r.t;

    % Positions-KPIs (phasen-vergleichbar)
    kpi.rmse_ep = sqrt(mean(ep.^2));
    kpi.mae_ep  = mean(ep);
    kpi.max_ep  = max(ep);
    n = numel(ep);
    tail = max(1, round(0.9*n)) : n;     % letzte 10 % ~ Endfehler
    kpi.final_ep = mean(ep(tail));

    % Effort-/Smoothness-KPI (NUR bei gleicher Dauer+Derating vergleichbar)
    if ~isempty(r.dq_cmd) && size(r.dq_cmd,1) >= 2
        ddq = diff(r.dq_cmd, 1, 1);
        dt  = diff(t); dt(dt <= 0) = NaN;
        kpi.cmd_rate = mean(vecnorm(ddq,2,2) ./ dt, 'omitnan');
    else
        kpi.cmd_rate = NaN;
    end

    % Completion
    kpi.duration = t(end) - t(1);
    kpi.phaseEnd = min(1, (t(end) - t(1)) / max(r.Tref, eps));  % 1.0 = voll gefahren
end

function agg = aggregateKPIs(kpis)
% Mittelwert/Std pro KPI-Feld ueber Runs einer Quelle.
    agg = struct(); agg.n = numel(kpis);
    if isempty(kpis), return; end
    fn = fieldnames(kpis);
    for f = 1:numel(fn)
        v = [kpis.(fn{f})];
        agg.(fn{f}).mean = mean(v, 'omitnan');
        agg.(fn{f}).std  = std(v, 0, 'omitnan');
    end
end

function printComparisonTable(aggSim, aggReal)
    rows = { ...
        'rmse_ep',  'RMSE ||e_p|| [m]',        true; ...
        'mae_ep',   'MAE  ||e_p|| [m]',        true; ...
        'max_ep',   'Max  ||e_p|| [m]',        true; ...
        'final_ep', 'Endfehler ||e_p|| [m]',   true; ...
        'cmd_rate', 'Kommando-Rate [rad/s^2]', false; ...
        'phaseEnd', 'Phase erreicht [-]',      false };

    fprintf('\n===================== Sim-to-Real KPI-Vergleich =====================\n');
    fprintf('%-26s | %-16s | %-16s | %-10s | vgl\n', ...
            'KPI', 'Sim (mu+/-sd)', 'Real (mu+/-sd)', 'Delta(R-S)');
    fprintf('%s\n', repmat('-', 1, 84));
    for i = 1:size(rows,1)
        key = rows{i,1}; lab = rows{i,2}; cmp = rows{i,3};
        s = getStat(aggSim,  key);
        r = getStat(aggReal, key);
        if isnan(s.mean) || isnan(r.mean), d = NaN; else, d = r.mean - s.mean; end
        fprintf('%-26s | %7.4f+/-%-6.4f | %7.4f+/-%-6.4f | %+9.4f | %s\n', ...
                lab, s.mean, s.std, r.mean, r.std, d, ternary(cmp,'ja','BEDINGT'));
    end
    fprintf('%s\n', repmat('-', 1, 84));
    fprintf(['Hinweis: "BEDINGT" = nur vergleichbar, wenn Sim- und Real-Dauer\n' ...
             '         sowie das 0.35-Derating im Deploy angeglichen sind.\n']);
    fprintf('=====================================================================\n');
end

function [eeX, eeZ, epN] = resampleGroup(runs, sgrid)
% Resampled ee(x), ee(z), ||ep|| jeder Episode auf das Phasen-Gitter.
    ng = numel(sgrid); m = numel(runs);
    eeX = nan(ng, m); eeZ = nan(ng, m); epN = nan(ng, m);
    for k = 1:m
        [ph, iu] = uniqueIncreasing(phaseOf(runs(k)));
        eeX(:,k) = interp1(ph, runs(k).ee(iu,1),    sgrid, 'linear', NaN);
        eeZ(:,k) = interp1(ph, runs(k).ee(iu,3),    sgrid, 'linear', NaN);
        epN(:,k) = interp1(ph, runs(k).ep_norm(iu), sgrid, 'linear', NaN);
    end
end

function [refX, refZ] = resampleRef(runs, sgrid)
% Referenzpfad auf Phase (geometrisch identisch sim/real).
    refX = nan(numel(sgrid),1); refZ = refX;
    for k = 1:numel(runs)
        if ~isempty(runs(k).ee_ref) && ~all(isnan(runs(k).ee_ref(:)))
            [ph, iu] = uniqueIncreasing(phaseOf(runs(k)));
            refX = interp1(ph, runs(k).ee_ref(iu,1), sgrid, 'linear', 'extrap');
            refZ = interp1(ph, runs(k).ee_ref(iu,3), sgrid, 'linear', 'extrap');
            return;
        end
    end
end

function ph = phaseOf(r)
    ph = (r.t - r.t(1)) / max(r.Tref, eps);
end

function [xu, iu] = uniqueIncreasing(x)
    [xu, iu] = unique(x(:), 'stable');
    [xu, o]  = sort(xu); iu = iu(o);
end

function plotBand(sgrid, M, rgb)
    mu = mean(M, 2, 'omitnan'); sd = std(M, 0, 2, 'omitnan');
    good = ~isnan(mu);
    s = sgrid(good); mu = mu(good); sd = sd(good);
    fill([s; flipud(s)], [mu+sd; flipud(mu-sd)], rgb, ...
         'FaceAlpha', 0.15, 'EdgeColor', 'none');
    plot(s, mu, 'Color', rgb, 'LineWidth', 1.6);
end

function plotKpiBars(aggSim, aggReal, keys, labels, ttl)
    nb = numel(keys);
    mu = nan(nb,2); sd = nan(nb,2);
    for i = 1:nb
        s = getStat(aggSim, keys{i}); r = getStat(aggReal, keys{i});
        mu(i,:) = [s.mean, r.mean]; sd(i,:) = [s.std, r.std];
    end
    figure('Name', ['sim2real: ' ttl]);
    x      = (1:nb).';
    colors = [0.2 0.4 0.9; 0.9 0.3 0.3];   % Sim / Real
    nbars  = 2; gw = 0.8; bw = gw / nbars;
    hold on;
    hbAll = gobjects(nbars,1);
    for j = 1:nbars
        xj = x - gw/2 + (j-0.5)*bw;         % gruppierte Position pro Quelle
        % Jede Quelle als EIGENER bar-Aufruf -> funktioniert auch bei nur 1 KPI.
        hbAll(j) = bar(xj, mu(:,j), bw, 'FaceColor', colors(j,:));
        errorbar(xj, mu(:,j), sd(:,j), 'k', 'linestyle', 'none', 'LineWidth', 1);
    end
    set(gca, 'XTick', 1:nb, 'XTickLabel', labels);
    legend(hbAll, {'Sim','Real'}, 'Location', 'best'); grid on;
    title(ttl); xlim([0.5, nb+0.5]);
end

function writeSummaryCsv(fpath, runs, kpis)
    fid = fopen(fpath, 'w');
    fprintf(fid, 'label,source,rmse_ep,mae_ep,max_ep,final_ep,cmd_rate,duration,phaseEnd\n');
    for i = 1:numel(runs)
        k = kpis(i);
        fprintf(fid, '%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f\n', ...
                runs(i).label, runs(i).source, k.rmse_ep, k.mae_ep, k.max_ep, ...
                k.final_ep, k.cmd_rate, k.duration, k.phaseEnd);
    end
    fclose(fid);
end

function s = getStat(agg, key)
    if isfield(agg, key), s = agg.(key);
    else, s = struct('mean', NaN, 'std', NaN); end
end

function u = unitOf(meta, field, fallback)
    u = string(fallback);
    if isfield(meta,'units') && isfield(meta.units, field)
        u = string(meta.units.(field));
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end
%% evaluate_p2p_hardware.m
% Reine HARDWARE-Auswertung der realen Punkt-zu-Punkt-Runs (p2p).
%
% KEIN Sim-Vergleich: die realen p2p-Runs verwenden unterschiedliche Start-
% und Zielpunkte, daher wird jeder Run GEGEN SEIN EIGENES ZIEL ausgewertet
% und nur ziel-relativ/dimensionslos aggregiert.
%
% Metrik-Konvention uebernommen aus calculate_kpi_spacekinova_point.m
% (Reaching-KPIs), reduziert auf das, was reale Hardware liefert:
%   - Endfehler, Min-Distanz, Setzzeit (in Toleranz + Haltezeit), Erfolg
%   - Pfadlaenge (pro Run) + Pfad-Effizienz (aggregierbar, dimensionslos)
%   - Overshoot nach Erstkontakt mit der Toleranz
%   - optional: Kommando-Rate aus dq_cmd (HW-Glaettungs-Proxy)
% NICHT enthalten (real nicht vorhanden): Basis-Stoerung, tau-Leistung/-Jerk,
% Reward -- feste Basis, Geschwindigkeitsschnittstelle, kein Reward-Signal.
%
% Aufruf: als Skript ausfuehren, cfg unten anpassen.

clc; clear; close all;

%% =========================
%  KONFIGURATION
%  =========================
cfg = struct();
cfg.runDir       = sk_path("data", "hardware", "runs");   % Ordner mit run_*.mat
cfg.pattern      = "run_*.mat";
cfg.includeLabel = "p2p";         % NUR Dateien mit diesem Tag im Namen
cfg.eeSource     = "fk";          % "fk" (Modell-EE) | "kortex" (HW-Wahrheit)

% --- Reaching-Toleranzen (wie im Sim-Point-Skript) ---
cfg.tol_success  = 0.05;          % Distanz [m] fuer "Ziel erreicht"
cfg.tol_settle   = 0.05;          % Distanz [m] fuer Setzzeit-Schwelle
cfg.settle_hold  = 0.5;           % Mindestverweildauer in Toleranz [s]

% --- Einheiten / Speichern ---
cfg.dqRealUnits  = "deg";         % Fallback-Einheit dq_cmd (wenn meta.units fehlt)
cfg.saveSummary  = true;

%% =========================
%  1) p2p-RUNS LADEN (nur real)
%  =========================
files = dir(fullfile(cfg.runDir, cfg.pattern));
assert(~isempty(files), "Keine Runs gefunden in %s/%s", cfg.runDir, cfg.pattern);

runsCell = {};
for i = 1:numel(files)
    nm = string(files(i).name);
    if ~contains(lower(nm), lower(cfg.includeLabel))
        continue;                       % nicht-p2p ueberspringen
    end
    fpath = fullfile(files(i).folder, files(i).name);
    try
        r = loadRunP2P(fpath, cfg);
    catch ME
        fprintf("  [SKIP-FEHLER] %s: %s\n", nm, ME.message);
        continue;
    end
    if r.source ~= "real"
        fprintf("  [SKIP] %s ist source='%s' (nur real wird ausgewertet)\n", nm, r.source);
        continue;
    end
    runsCell{end+1} = r; %#ok<AGROW>
end
assert(~isempty(runsCell), "Keine realen p2p-Runs gefunden.");
runs = [runsCell{:}];

fprintf("Geladen: %d reale p2p-Runs (EE-Quelle: %s)\n", numel(runs), cfg.eeSource);

%% =========================
%  2) PRO-RUN KPIs
%  =========================
kpiAll = arrayfun(@(r) computeP2PKpis(r, cfg), runs);

%% =========================
%  3) PRO-RUN TABELLE
%  =========================
fprintf('\n==================================== p2p Hardware-Auswertung (pro Run) ====================================\n');
fprintf('%-5s | %-22s | %7s | %7s | %7s | %7s | %5s | %8s | %5s | %7s\n', ...
        'ID','Label','d0[mm]','fin[mm]','min[mm]','settle','succ','path[mm]','eff','over[mm]');
fprintf('%s\n', repmat('-', 1, 106));
for i = 1:numel(runs)
    k = kpiAll(i);
    settleStr = ternary(isnan(k.settle), '   -  ', sprintf('%5.2fs', k.settle));
    fprintf('%-5s | %-22s | %7.1f | %7.1f | %7.1f | %6s | %5s | %8.1f | %5.2f | %7.1f\n', ...
            runs(i).id, runs(i).label, k.d0*1000, k.finalErr*1000, k.minErr*1000, ...
            settleStr, ternary(k.success,'JA','--'), k.pathLen*1000, k.pathEff, k.overshoot*1000);
end
fprintf('%s\n', repmat('-', 1, 106));

%% =========================
%  4) AGGREGAT (nur ziel-relativ/dimensionslos)
%  =========================
finalErr = [kpiAll.finalErr]; minErr = [kpiAll.minErr];
settle   = [kpiAll.settle];   success = [kpiAll.success];
pathEff  = [kpiAll.pathEff];  overshoot = [kpiAll.overshoot];
cmdRate  = [kpiAll.cmdRate];

fprintf('\n========= Aggregat ueber %d reale p2p-Runs =========\n', numel(runs));
fprintf('  Erfolgsrate (<%.0f mm)   : %.0f %% (%d/%d)\n', cfg.tol_success*1000, ...
        100*mean(success), sum(success), numel(success));
fprintf('  Endfehler              : %.1f mm (mean) / %.1f mm (max) / %.1f mm (std)\n', ...
        mean(finalErr)*1000, max(finalErr)*1000, std(finalErr)*1000);
fprintf('  Min-Distanz            : %.1f mm (mean)\n', mean(minErr)*1000);
if any(~isnan(settle))
    fprintf('  Setzzeit               : %.2f s (mean ueber gesetzte) / %.0f %% gesetzt\n', ...
            mean(settle,'omitnan'), 100*mean(~isnan(settle)));
else
    fprintf('  Setzzeit               : kein Run innerhalb Toleranz gehalten\n');
end
fprintf('  Pfad-Effizienz         : %.2f (mean, 1.0 = direkt)\n', mean(pathEff));
fprintf('  Overshoot              : %.1f mm (mean)\n', mean(overshoot)*1000);
if any(~isnan(cmdRate))
    fprintf('  Kommando-Rate          : %.3f rad/s^2 (mean, HW-Glaettungs-Proxy)\n', mean(cmdRate,'omitnan'));
end
fprintf('====================================================\n');

%% =========================
%  5) PLOTS
%  =========================
cmap = lines(numel(runs));

% (a) Distanz zum Ziel ueber Zeit -- alle Runs konvergieren gegen ihr eigenes Ziel
figure('Name','p2p: Distanz zum Ziel ueber Zeit','Position',[100 100 820 480]);
hold on; grid on; leg = {};
for i = 1:numel(runs)
    [t, d] = distOf(runs(i));
    plot(t - t(1), d*1000, '-', 'Color', cmap(i,:), 'LineWidth', 1.3);
    leg{end+1} = runs(i).id; %#ok<AGROW>
end
yline(cfg.tol_success*1000, 'k--', sprintf('Toleranz %.0f mm', cfg.tol_success*1000), 'LineWidth', 1.2);
xlabel('Zeit seit Start [s]'); ylabel('Distanz zum (eigenen) Ziel [mm]');
title('p2p-Konvergenz pro Hardware-Run');
legend(leg, 'Location', 'northeast');

% (b) Endfehler pro Run als Balken, Erfolgsschwelle markiert
figure('Name','p2p: Endfehler pro Run','Position',[100 100 720 420]);
fe = finalErr*1000;
b = bar(fe, 'FaceColor', 'flat'); hold on; grid on;
for i = 1:numel(runs)
    if success(i), b.CData(i,:) = [0.2 0.6 0.3]; else, b.CData(i,:) = [0.8 0.3 0.3]; end
end
yline(cfg.tol_success*1000, 'k--', 'Erfolgs-Toleranz', 'LineWidth', 1.2);
set(gca, 'XTick', 1:numel(runs), 'XTickLabel', {runs.id}, 'XTickLabelRotation', 30);
ylabel('Endfehler [mm]'); title('Endfehler pro p2p-Run (gruen = Erfolg)');

% (c) 3D-Pfade, jeder Run mit eigenem Start (o) und Ziel (*)
figure('Name','p2p: EE-Pfade 3D','Position',[100 100 760 600]);
hold on; grid on;
for i = 1:numel(runs)
    ee = runs(i).ee; tg = runs(i).target; st = ee(1,:);
    plot3(ee(:,1), ee(:,2), ee(:,3), '-', 'Color', cmap(i,:), 'LineWidth', 1.2);
    plot3(st(1), st(2), st(3), 'o', 'Color', cmap(i,:), 'MarkerFaceColor', cmap(i,:), 'MarkerSize', 6);
    plot3(tg(1), tg(2), tg(3), 'p', 'Color', cmap(i,:), 'MarkerFaceColor', cmap(i,:), 'MarkerSize', 12);
end
axis equal; view(45,25);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('EE-Pfade je Run  (o = Start, \star = Ziel)');

%% =========================
%  6) SUMMARY SPEICHERN
%  =========================
if cfg.saveSummary
    stamp = string(datetime('now'), 'yyyyMMdd_HHmmss');
    summary = struct('cfg', cfg, 'perRun', kpiAll, 'ids', {{runs.id}});
    save(fullfile(cfg.runDir, sprintf("p2p_hw_summary_%s.mat", stamp)), '-struct', 'summary');
    fid = fopen(fullfile(cfg.runDir, sprintf("p2p_hw_kpis_%s.csv", stamp)), 'w');
    fprintf(fid, 'id,label,d0_m,finalErr_m,minErr_m,settle_s,success,pathLen_m,pathEff,overshoot_m,cmdRate\n');
    for i = 1:numel(runs)
        k = kpiAll(i);
        fprintf(fid, '%s,%s,%.6f,%.6f,%.6f,%.4f,%d,%.6f,%.4f,%.6f,%.6f\n', ...
                runs(i).id, runs(i).label, k.d0, k.finalErr, k.minErr, k.settle, ...
                k.success, k.pathLen, k.pathEff, k.overshoot, k.cmdRate);
    end
    fclose(fid);
    fprintf("\nSummary gespeichert in %s/\n", cfg.runDir);
end

%% =====================================================================
%  LOKALE FUNKTIONEN
%  =====================================================================

function r = loadRunP2P(fpath, cfg)
% Laedt einen p2p-Run und liefert kanonisches Schema fuer die Reaching-Auswertung.
    S = load(fpath);
    if isfield(S,'data') && isfield(S,'meta')
        data = S.data; meta = S.meta;
    elseif isfield(S,'run') && isfield(S.run,'data')
        data = S.run.data; meta = S.run.meta;
    else
        error("Unbekanntes Run-Format (kein data/meta).");
    end

    source = lower(string(meta.source));

    if     isfield(data,'t'),     t = data.t(:);
    elseif isfield(data,'t_ref'), t = data.t_ref(:);
    else,  error("Keine Zeitbasis (t / t_ref)."); end

    if source == "real" && cfg.eeSource == "kortex" ...
            && isfield(data,'ee_kortex') && ~all(isnan(data.ee_kortex(:)))
        ee = data.ee_kortex;
    else
        ee = data.ee_measured;
    end

    % Ziel = konstanter ee_ref (p2p). Letzter Wert ist robust gegen evtl. Anfahrprofil.
    assert(isfield(data,'ee_ref'), "ee_ref fehlt -- kann Zielpunkt nicht bestimmen.");
    target = data.ee_ref(end,:);

    % dq_cmd -> rad/s (optional)
    dq = [];
    if isfield(data,'dq_cmd')
        dq = data.dq_cmd;
        u = cfg.dqRealUnits;
        if isfield(meta,'units') && isfield(meta.units,'dq_cmd'), u = string(meta.units.dq_cmd); end
        if string(u) == "deg", dq = deg2rad(dq); end
    end

    % ID aus Dateiname (run_NNN_...), Labels sind nicht eindeutig
    [~, base] = fileparts(fpath);
    tok = regexp(base, '^(run_\d+)', 'tokens', 'once');
    if isempty(tok), id = string(base); else, id = string(tok{1}); end
    if isfield(meta,'label'), label = string(meta.label); else, label = "?"; end

    r = struct('source', source, 'id', id, 'label', label, ...
               't', t, 'ee', ee, 'target', target, 'dq_cmd', dq);
end

function [t, d] = distOf(r)
% Distanz zum (eigenen) Ziel ueber die Zeit.
    t = r.t;
    d = vecnorm(r.ee - r.target, 2, 2);
end

function kpi = computeP2PKpis(r, cfg)
% Reaching-KPIs gegen das run-eigene Ziel.
    [t, dist] = distOf(r);
    ee = r.ee; target = r.target; st = ee(1,:);

    kpi.d0       = norm(target - st);     % Startdistanz zum Ziel
    kpi.finalErr = dist(end);
    kpi.minErr   = min(dist);
    kpi.success  = dist(end) < cfg.tol_success;

    % --- Setzzeit: erstes Eintreten in Toleranz, das >= settle_hold haelt ---
    kpi.settle = NaN;
    in_tol = dist < cfg.tol_settle;
    if any(in_tol)
        idx_in = find(in_tol);
        for ii = 1:numel(idx_in)
            t0   = t(idx_in(ii));
            mask = t >= t0 & t <= min(t0 + cfg.settle_hold, t(end));
            if all(dist(mask) < cfg.tol_settle) && (t(end) - t0) >= cfg.settle_hold
                kpi.settle = t0 - t(1);
                break;
            end
        end
    end

    % --- Pfadlaenge + Effizienz ---
    kpi.pathLen = 0; kpi.pathEff = 0;
    if size(ee,1) >= 2
        kpi.pathLen = sum(vecnorm(diff(ee), 2, 2));
        if kpi.pathLen > 1e-6
            kpi.pathEff = min(1, kpi.d0 / kpi.pathLen);   % direkt / tatsaechlich
        end
    end

    % --- Overshoot: max Distanz nach Erstkontakt mit Toleranz ---
    kpi.overshoot = 0;
    if any(in_tol)
        i0 = find(in_tol, 1, 'first');
        if i0 < numel(dist)
            kpi.overshoot = max(0, max(dist(i0:end)) - cfg.tol_settle);
        end
    end

    % --- Kommando-Rate (HW-Glaettungs-Proxy, nicht tau-Jerk) ---
    kpi.cmdRate = NaN;
    if ~isempty(r.dq_cmd) && size(r.dq_cmd,1) >= 2
        ddq = diff(r.dq_cmd, 1, 1);
        dt  = diff(t); dt(dt <= 0) = NaN;
        kpi.cmdRate = mean(vecnorm(ddq,2,2) ./ dt, 'omitnan');
    end
end

function out = ternary(cond, a, b)
    if cond, out = a; else, out = b; end
end

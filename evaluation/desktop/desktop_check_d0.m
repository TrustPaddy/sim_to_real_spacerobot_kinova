function T = desktop_check_d0()
%DESKTOP_CHECK_D0  Prueft das parametrisierte Modell SK_desktop gegen die Originalmodelle (Desktop-Plan D0).
%   T = desktop_check_d0()
%
%   Paare (Original gegen SK_desktop mit denselben Einstellungen, deterministische Policy):
%     A  40-Hz-Modell mit CDR2-4             gegen SK_desktop 40 Hz, 65 kg, ohne Verzoegerung
%     B  CDR-Modell (Teststand, A23) mit CDR2-4 gegen SK_desktop 40 Hz, 16,25 kg, 2 Schritte, Daempfung x2
%     C  10-Hz-Modell mit ppo_10hz gegen SK_desktop 10 Hz, Solver 20 ms, 65 kg. Das 10-Hz-Modell rechnet
%        den Reward im Solver-Takt (20 ms), SK_desktop im Agenten-Takt. Der Return ist deshalb nicht
%        vergleichbar, die Bewegung schon (der Abbruch ist im 10-Hz-Modell auskommentiert, A25)
%   Weitere Pruefungen mit SK_desktop:
%     D  Wiederholung von A (deterministisch, muss identisch sein)
%     E  ppo_10hz mit Solver 5 ms statt 20 ms (Einfluss des Solver-Schritts, D1 nutzt 5 ms)
%     H  ppo_10hz mit 6,5 kg wie im Arbeitsstand vom 19.06. (A33)
%     F  CDR2-4 mit 1e9 kg (Basis praktisch fest, Grundlage fuer D3)
%     G  CDR2-4 mit stochastischer Policy wie in den alten KPI-Skripten (A28)
%   Ergebnis: data/simulation/desktop/D0_check_<Zeit>.mat und .csv

setup_project;
desktop_build_model(true);   % Modell immer frisch aus der Quelle bauen

aCdr = sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat');
a10  = sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'ppo_10hz.mat');

base40 = {'agentFile', aCdr, 'agentLabel', 'CDR2-4', 'Ts', 0.005, 'Ts_agent', 0.025};
base10 = {'agentFile', a10, 'agentLabel', 'ppo_10hz', 'Ts', 0.02, 'Ts_agent', 0.1};

runs = {
    'A_orig40',   desktop_config(base40{:}, 'model', 'SpaceKinova_MotionProfile_40Hz')
    'A_desk40',   desktop_config(base40{:}, 'base_mass', 65)
    'B_origCDR',  desktop_config(base40{:}, 'model', 'SpaceKinova_MotionProfile_CDR')
    'B_deskCDR',  desktop_config(base40{:}, 'base_mass', 65 * 0.25, 'delay_steps', 2, 'damp_scale', 2)
    'C_orig10',   desktop_config(base10{:}, 'model', 'SpaceKinova_MotionProfile')
    'C_desk10',   desktop_config(base10{:}, 'base_mass', 65)
    'D_desk40_rep', desktop_config(base40{:}, 'base_mass', 65)
    'E_desk10_5ms', desktop_config(base10{:}, 'base_mass', 65, 'Ts', 0.005)
    'F_desk40_fixed', desktop_config(base40{:}, 'base_mass', 1e9)
    'G_desk40_stoch', desktop_config(base40{:}, 'base_mass', 65, 'explore', true, 'seed', 1)
    'H_desk10_6p5kg', desktop_config(base10{:}, 'base_mass', 65 * 0.1)
    };

% Basismasse der Originalmodelle festhalten
origMass = struct();
for m = {'SpaceKinova_MotionProfile_40Hz', 'SpaceKinova_MotionProfile_CDR', 'SpaceKinova_MotionProfile'}
    load_system(m{1});
    origMass.(m{1}) = get_param([m{1} '/Robot/base_link/Inertia'], 'Mass');
end

res = cell(size(runs, 1), 1);
for i = 1:size(runs, 1)
    c = runs{i, 2};
    c.label = runs{i, 1};
    fprintf('[%d/%d] %s ...\n', i, size(runs, 1), c.label);
    res{i} = desktop_run_episode(c);
    mm = res{i}.metrics;
    fprintf('    MSE %.3e m^2, max %.4f m, Return %.1f, Stopp %s, %.1f s\n', ...
        mm.mse_ee, mm.max_ee, mm.ret, mm.stop_reason, res{i}.info.wallTime);
end

% --- Tabelle ---
lab = runs(:, 1);
gm = @(f) cellfun(@(r) r.metrics.(f), res);
T = table(lab, cellfun(@(r) r.cfg.model, res, 'UniformOutput', false), ...
    cellfun(@(r) r.cfg.agentLabel, res, 'UniformOutput', false), ...
    cellfun(@(r) 1 / r.cfg.Ts_agent, res), cellfun(@(r) r.cfg.Ts, res), ...
    cellfun(@(r) r.cfg.base_mass, res), cellfun(@(r) r.cfg.delay_steps, res), ...
    gm('mse_ee'), gm('rms_ee'), gm('max_ee'), gm('ret'), gm('early_stop'), ...
    cellfun(@(r) r.metrics.stop_reason, res, 'UniformOutput', false), gm('n_steps'), ...
    gm('ori_max'), gm('w_mean'), gm('sat_frac_J6'), gm('araw_absmax_J6'), ...
    cellfun(@(r) r.info.wallTime, res), ...
    'VariableNames', {'case', 'model', 'agent', 'rate_Hz', 'Ts', 'base_mass', 'delay_steps', ...
    'mse_ee', 'rms_ee', 'max_ee', 'return', 'early_stop', 'stop_reason', 'n_steps', ...
    'ori_max', 'w_mean', 'sat_frac_J6', 'araw_absmax_J6', 'wall_s'});
disp(T);

% --- Paarvergleiche ---
pairs = {'A_orig40', 'A_desk40'; 'B_origCDR', 'B_deskCDR'; 'C_orig10', 'C_desk10'; ...
    'A_desk40', 'D_desk40_rep'; 'C_desk10', 'E_desk10_5ms'; 'A_desk40', 'F_desk40_fixed'; ...
    'A_desk40', 'G_desk40_stoch'; 'C_desk10', 'H_desk10_6p5kg'};
cmp = struct('a', {}, 'b', {}, 'max_dep', {}, 'd_mse_rel', {}, 'd_ret', {});
fprintf('\nPaarvergleich (max. Abweichung der EE-Fehlervektoren auf gemeinsamer Zeitachse):\n');
for p = 1:size(pairs, 1)
    ra = res{strcmp(lab, pairs{p, 1})};
    rb = res{strcmp(lab, pairs{p, 2})};
    tEnd = min(ra.ts.t_ee(end), rb.ts.t_ee(end));
    tg = (0:0.005:tEnd).';
    ea = interp1(ra.ts.t_ee, ra.ts.ep, tg, 'linear');
    eb = interp1(rb.ts.t_ee, rb.ts.ep, tg, 'linear');
    c = struct('a', pairs{p, 1}, 'b', pairs{p, 2}, 'max_dep', max(vecnorm(ea - eb, 2, 2)), ...
        'd_mse_rel', (rb.metrics.mse_ee - ra.metrics.mse_ee) / ra.metrics.mse_ee, ...
        'd_ret', rb.metrics.ret - ra.metrics.ret);
    cmp(end + 1) = c; %#ok<AGROW>
    fprintf('  %-12s gegen %-15s  max|dep| = %.3e m, MSE %+6.2f %%, Return %+8.2f\n', ...
        c.a, c.b, c.max_dep, 100 * c.d_mse_rel, c.d_ret);
end

% --- Speichern ---
outDir = sk_path('data', 'simulation', 'desktop');
if ~isfolder(outDir), mkdir(outDir); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
save(fullfile(outDir, ['D0_check_' stamp '.mat']), 'T', 'cmp', 'res', 'origMass', '-v7.3');
writetable(T, fullfile(outDir, ['D0_check_' stamp '.csv']));
writetable(struct2table(cmp), fullfile(outDir, ['D0_check_' stamp '_pairs.csv']));
fprintf('\nBasismasse der Originalmodelle: 40Hz %s | CDR %s | 10Hz %s\n', ...
    origMass.SpaceKinova_MotionProfile_40Hz, origMass.SpaceKinova_MotionProfile_CDR, ...
    origMass.SpaceKinova_MotionProfile);
fprintf('Gespeichert: %s\n', fullfile(outDir, ['D0_check_' stamp '.*']));
end

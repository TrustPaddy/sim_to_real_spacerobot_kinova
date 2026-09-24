function [E, S] = desktop_d1_rates(nDraw)
%DESKTOP_D1_RATES  Ratenversuch 2x2 in Simulation (Desktop-Plan D1, R1.6, A9, A33, A44).
%   [E, S] = desktop_d1_rates()     volle Reihe (20 Ziehungen je Satz)
%   [E, S] = desktop_d1_rates(2)    Kurztest
%
%   Jeder Agent laeuft bei 40 Hz und bei 10 Hz im selben Modell (SK_desktop, Solver 5 ms, Halbkreis 8,5 s,
%   Referenz nach Wandzeit). Filter und Rate Limiter rechnen im Agenten-Takt, wie auf der Hardware.
%   Agenten:
%     CDR2-4     40 Hz, Bayes-Hyperparameter und CDR (Hardwarelaeufe 012-014)
%     Optimized  40 Hz, Bayes-Hyperparameter ohne CDR
%     PPO_base   40 Hz, eigene Hyperparameter, vor Bayes und CDR (Hardwarelaeufe 008-011, A46)
%     ppo_10hz   10 Hz, ohne Bayes und CDR (A21)
%   Saetze:
%     nominal    65 kg, ohne Verzoegerung, deterministisch. ppo_10hz zusaetzlich mit 6,5 kg (A33)
%     stoch      wie nominal, stochastische Policy, Seeds 1..nDraw (Vergleich mit den alten KPI-Werten, A28)
%     mass       deterministisch, Basismasse 65 kg * max(0,5; 1 + 0,65 randn) wie CDR-Phase 4, nDraw Ziehungen,
%                fuer alle Agenten und Raten dieselben Ziehungen
%     delay      deterministisch, 65 kg, Verzoegerung 0,1 s und 0,2 s (4 und 8 Schritte bei 40 Hz,
%                1 und 2 Schritte bei 10 Hz)
%     e1         CDR2-4 bei 40 Hz unter der kombinierten Stoerung aus E1 (16,25 kg, 2 Schritte, Daempfung x2),
%                deterministisch und stochastisch (Vergleich mit 0,0049 m^2)
%   Ergebnis: data/simulation/desktop/D1_rates_<Zeit>.mat, _episodes.csv, _summary.csv

if nargin < 1, nDraw = 20; end
setup_project;
desktop_build_model();

ag = struct( ...
    'label', {'CDR2-4', 'Optimized', 'PPO_base', 'ppo_10hz'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'SpaceKinova_PPO_agent_motionprofile.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'ppo_10hz.mat')}, ...
    'trainHz', {40, 40, 40, 10});
rates = [40 10];

% Massenziehungen (fest, fuer alle Agenten gleich)
rng(20260924, 'twister');
massFactor = max(0.5, 1 + 0.65 * randn(nDraw, 1));

jobs = {};
for a = 1:numel(ag)
    for hz = rates
        base = {'agentFile', ag(a).file, 'agentLabel', ag(a).label, 'Ts', 0.005, 'Ts_agent', 1 / hz, ...
            'base_mass', 65, 'keepTs', false};
        jobs(end + 1, :) = {'nominal', 0, desktop_config(base{:}, 'keepTs', true)}; %#ok<AGROW>
        if strcmp(ag(a).label, 'ppo_10hz')
            jobs(end + 1, :) = {'nominal_6p5kg', 0, desktop_config(base{:}, 'base_mass', 6.5, 'keepTs', true)}; %#ok<AGROW>
        end
        for k = 1:nDraw
            jobs(end + 1, :) = {'stoch', k, desktop_config(base{:}, 'explore', true, 'seed', k)}; %#ok<AGROW>
        end
        for k = 1:nDraw
            jobs(end + 1, :) = {'mass', k, desktop_config(base{:}, 'base_mass', 65 * massFactor(k))}; %#ok<AGROW>
        end
        for d = [0.1 0.2]
            jobs(end + 1, :) = {sprintf('delay_%03dms', round(1000 * d)), 0, ...
                desktop_config(base{:}, 'delay_steps', round(d * hz))}; %#ok<AGROW>
        end
    end
end
e1 = {'agentFile', ag(1).file, 'agentLabel', ag(1).label, 'Ts', 0.005, 'Ts_agent', 0.025, ...
    'base_mass', 16.25, 'delay_steps', 2, 'damp_scale', 2, 'keepTs', false};
jobs(end + 1, :) = {'e1', 0, desktop_config(e1{:})};
for k = 1:nDraw
    jobs(end + 1, :) = {'e1_stoch', k, desktop_config(e1{:}, 'explore', true, 'seed', k)}; %#ok<AGROW>
end

% Jobs nach Masse sortieren spart Neukompilierungen
masses = cellfun(@(c) c.base_mass, jobs(:, 3));
[~, order] = sort(masses);
jobs = jobs(order, :);

n = size(jobs, 1);
fprintf('D1: %d Episoden\n', n);
res = cell(n, 1);
tAll = tic;
for i = 1:n
    c = jobs{i, 3};
    c.label = sprintf('%s_%s_%dHz_%02d', jobs{i, 1}, c.agentLabel, round(1 / c.Ts_agent), jobs{i, 2});
    try
        res{i} = desktop_run_episode(c);
    catch err
        warning('D1:episode', '%s fehlgeschlagen: %s', c.label, err.message);
        res{i} = struct('cfg', c, 'metrics', struct(), 'ts', struct(), 'info', struct('error', err.message));
    end
    if mod(i, 25) == 0 || i == n
        fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll));
    end
end

% --- Episodentabelle ---
rows = cell(n, 1);
for i = 1:n
    c = res{i}.cfg; m = res{i}.metrics;
    r = struct('set', jobs{i, 1}, 'draw', jobs{i, 2}, 'agent', c.agentLabel, ...
        'train_Hz', ag(strcmp({ag.label}, c.agentLabel)).trainHz, 'run_Hz', round(1 / c.Ts_agent), ...
        'base_mass', c.base_mass, 'delay_s', c.delay_steps * c.Ts_agent, 'explore', c.explore, ...
        'mse_ee', NaN, 'rms_ee', NaN, 'max_ee', NaN, 'completed', false, 'stop_reason', 'error', ...
        't_end', NaN, 'ori_max', NaN, 'w_mean', NaN, 'sat_frac_J2', NaN, 'sat_frac_J4', NaN, ...
        'sat_frac_J6', NaN, 'qlim_frac', NaN, 'ret', NaN);
    if isfield(m, 'mse_ee')
        for f = {'mse_ee', 'rms_ee', 'max_ee', 'stop_reason', 't_end', 'ori_max', 'w_mean', ...
                'sat_frac_J2', 'sat_frac_J4', 'sat_frac_J6', 'qlim_frac', 'ret'}
            r.(f{1}) = m.(f{1});
        end
        r.completed = ~m.early_stop;
    end
    rows{i} = r;
end
E = struct2table([rows{:}]);
E.matched = E.train_Hz == E.run_Hz;

% --- Zusammenfassung je Satz, Agent und Rate ---
E.setgroup = E.set;
E.setgroup(startsWith(E.set, 'delay')) = E.set(startsWith(E.set, 'delay'));
G = findgroups(E.setgroup, E.agent, E.run_Hz, E.base_mass .* strcmp(E.set, 'nominal_6p5kg'));
S = table();
for g = unique(G).'
    idx = G == g;
    sub = E(idx, :);
    s = table();
    s.set = sub.setgroup(1);
    s.agent = sub.agent(1);
    s.train_Hz = sub.train_Hz(1);
    s.run_Hz = sub.run_Hz(1);
    s.n = height(sub);
    s.completion = mean(sub.completed);
    s.mse_mean = mean(sub.mse_ee);
    s.mse_std = std(sub.mse_ee);
    s.mse_median = median(sub.mse_ee);
    s.rms_mean = mean(sub.rms_ee);
    s.max_mean = mean(sub.max_ee);
    s.max_max = max(sub.max_ee);
    s.mse_completed_mean = mean(sub.mse_ee(sub.completed));
    s.sat_J6 = mean(sub.sat_frac_J6);
    S = [S; s]; %#ok<AGROW>
end
S = sortrows(S, {'set', 'agent', 'run_Hz'});
disp(S);

outDir = sk_path('data', 'simulation', 'desktop');
if ~isfolder(outDir), mkdir(outDir); end
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if nDraw < 20, stamp = [stamp '_test']; end
save(fullfile(outDir, ['D1_rates_' stamp '.mat']), 'E', 'S', 'res', 'jobs', 'massFactor', 'ag', '-v7.3');
writetable(E(:, ~strcmp(E.Properties.VariableNames, 'setgroup')), fullfile(outDir, ['D1_rates_' stamp '_episodes.csv']));
writetable(S, fullfile(outDir, ['D1_rates_' stamp '_summary.csv']));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D1_rates_' stamp '.*']), toc(tAll) / 60);
end

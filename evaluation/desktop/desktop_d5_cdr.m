function [E, S] = desktop_d5_cdr(nSeeds)
%DESKTOP_D5_CDR  Vorhandene CDR-Agenten unter den Testbedingungen des Entwurfs E1 nachrechnen (Desktop-Plan D5).
%   [E, S] = desktop_d5_cdr()     volle Reihe (10 stochastische Seeds je Bedingung)
%   [E, S] = desktop_d5_cdr(1)    Kurztest
%
%   E1 (DigitalAgentAndRateMismatch/root.tex, Sec. "opt-cdr") vergleicht jeden CDR-Agenten mit einer Baseline
%   ohne CDR unter einer Stoerung, die nicht im Training vorkam:
%     CDR-1 auf dem gespiegelten Halbkreis, CDR-2 bei 25 % Basismasse, CDR-3 mit fester Verzoegerung 2 Schritte,
%     CDR-4 mit doppelter Gelenkdaempfung, CDR-2 bis CDR-4 unter allen drei Stoerungen zugleich.
%   Annahmen (im Code nicht dokumentiert):
%     - Baseline ohne CDR = Optimized.mat (Bayes-Hyperparameter, ohne CDR, 13.04., vor den CDR-Agenten)
%     - gespiegelter Halbkreis = x = cx - r sin statt cx + r sin (desktop_config 'mirror_x')
%   Alle Agenten laufen bei 40 Hz in SK_desktop, Basis 65 kg (ausser Masse-Bedingung), Halbkreis 8,5 s.
%   Je Agent und Bedingung eine deterministische Episode und nSeeds stochastische (wie die alten KPI-Skripte).
%   Ergebnis: data/simulation/desktop/D5_cdr_<Zeit>.mat, _episodes.csv, _summary.csv

if nargin < 1, nSeeds = 10; end
setup_project;
desktop_build_model();

d = sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO');
ag = struct('label', {'Optimized', 'Trajectory', 'Trajectory2', 'Mass_inertia', 'Actuator_delay', ...
    'Friction', 'CDR2-4', 'CDR1-4'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), ...
    fullfile(d, 'Trajectory.mat'), fullfile(d, 'Trajectory2.mat'), fullfile(d, 'Mass_inertia.mat'), ...
    fullfile(d, 'Actuator_delay.mat'), fullfile(d, 'Friction.mat'), fullfile(d, 'CDR2-4.mat'), ...
    fullfile(d, 'CDR1-4.mat')});

cond = struct('name', {'nominal', 'mirrored', 'mass25', 'delay2', 'damp2', 'combined'}, ...
    'mirror', {false, true, false, false, false, false}, ...
    'mass', {65, 65, 16.25, 65, 65, 16.25}, ...
    'delay', {0, 0, 0, 2, 0, 2}, ...
    'damp', {1, 1, 1, 1, 2, 2});

jobs = {};
for a = 1:numel(ag)
    for c = 1:numel(cond)
        base = {'agentFile', ag(a).file, 'agentLabel', ag(a).label, 'Ts_agent', 0.025, ...
            'mirror_x', cond(c).mirror, 'base_mass', cond(c).mass, 'delay_steps', cond(c).delay, ...
            'damp_scale', cond(c).damp, 'keepTs', false};
        jobs(end + 1, :) = {cond(c).name, 0, desktop_config(base{:})}; %#ok<AGROW>
        for k = 1:nSeeds
            jobs(end + 1, :) = {cond(c).name, k, desktop_config(base{:}, 'explore', true, 'seed', k)}; %#ok<AGROW>
        end
    end
end
key = cellfun(@(c) c.base_mass, jobs(:, 3));
[~, order] = sort(key);
jobs = jobs(order, :);

n = size(jobs, 1);
fprintf('D5: %d Episoden\n', n);
rows = cell(n, 1);
tAll = tic;
for i = 1:n
    c = jobs{i, 3};
    r = struct('cond', jobs{i, 1}, 'seed', jobs{i, 2}, 'agent', c.agentLabel, 'explore', c.explore, ...
        'K1_mse', NaN, 'K2_max', NaN, 'K3_ori_mean', NaN, 'K7_return', NaN, 'completed', false, ...
        'stop_reason', 'error');
    try
        res = desktop_run_episode(c);
        m = res.metrics;
        r.K1_mse = m.mse_ee; r.K2_max = m.max_ee; r.K3_ori_mean = m.ori_mean; r.K7_return = m.ret;
        r.completed = ~m.early_stop; r.stop_reason = m.stop_reason;
    catch err
        warning('D5:episode', '%s %s fehlgeschlagen: %s', c.agentLabel, jobs{i, 1}, err.message);
    end
    rows{i} = r;
    if mod(i, 50) == 0 || i == n, fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll)); end
end
E = struct2table([rows{:}]);

% Zusammenfassung: deterministisch und Mittel ueber die Seeds
S = table();
for a = {ag.label}
    for c = {cond.name}
        sub = E(strcmp(E.agent, a{1}) & strcmp(E.cond, c{1}), :);
        det = sub(~sub.explore, :); st = sub(sub.explore, :);
        s = table(a, c, det.K1_mse, det.K2_max, det.K3_ori_mean, det.K7_return, det.completed, ...
            mean(st.K1_mse), std(st.K1_mse), mean(st.K2_max), mean(st.K3_ori_mean), mean(st.K7_return), ...
            mean(st.completed), 'VariableNames', {'agent', 'cond', 'det_K1', 'det_K2', 'det_K3', 'det_K7', ...
            'det_completed', 'sto_K1', 'sto_K1_std', 'sto_K2', 'sto_K3', 'sto_K7', 'sto_completion'});
        S = [S; s]; %#ok<AGROW>
    end
end
disp(S);

outDir = sk_path('data', 'simulation', 'desktop');
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if nSeeds < 10, stamp = [stamp '_test']; end
save(fullfile(outDir, ['D5_cdr_' stamp '.mat']), 'E', 'S', 'jobs', 'ag', 'cond', '-v7.3');
writetable(E, fullfile(outDir, ['D5_cdr_' stamp '_episodes.csv']));
writetable(S, fullfile(outDir, ['D5_cdr_' stamp '_summary.csv']));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D5_cdr_' stamp '.*']), toc(tAll) / 60);
end

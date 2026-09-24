function [E, R] = desktop_d6_divergence(nPerAgent, nCdr)
%DESKTOP_D6_DIVERGENCE  Abbrueche durch NaN/Inf und grobe Divergenz, Verteilung des Schritt-Rewards (Desktop-Plan D6).
%   [E, R] = desktop_d6_divergence()        5 untrainierte Agenten x 40 Episoden, CDR2-4 mit 100 Episoden
%   [E, R] = desktop_d6_divergence(2, 2)    Kurztest
%
%   R1.1 fragt, warum die Simscape-Integration NaN/Inf zulaesst und wie oft das vorkommt. Im Training selbst ist
%   die Haeufigkeit nicht geloggt. Hier zwei Naeherungen im 40-Hz-Modell (SK_desktop, 65 kg, Halbkreis 8,5 s):
%     untrained  fuenf PPO-Agenten mit zufaelligen Gewichten (rlPPOAgent mit Standardnetzen, Seeds 1-5),
%                stochastische Policy. Das entspricht dem Anfang des Trainings.
%     cdr_p4     CDR2-4 stochastisch unter den Stoerungen der CDR-Phase 4: Basismasse 65 kg * max(0,5; 1 + 0,65 randn),
%                Verzoegerung 1-3 Schritte
%   Gezaehlt werden die Abbruchgruende (nonfinite, ep>0.5m, ev>2m/s, ori>1rad). Dazu die Verteilung des Rewards pro
%   Agentenschritt (ohne die -50 des Abbruchs). Die groesste geclippte Strafe pro Schritt ist
%   20*0,25 + 2*4 + 0,02*10 + 0,05*10 + 8*0,5 + 4*1 = 21,7.
%   Ergebnis: data/simulation/desktop/D6_divergence_<Zeit>.mat und _episodes.csv

if nargin < 1, nPerAgent = 40; end
if nargin < 2, nCdr = 100; end
setup_project;
desktop_build_model();
outDir = sk_path('data', 'simulation', 'desktop');

% Untrainierte Agenten mit derselben Beobachtung und Aktion wie CDR2-4
S = load(sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), 'agent');
oi = getObservationInfo(S.agent); ai = getActionInfo(S.agent);
files = cell(5, 1);
for k = 1:5
    rng(k, 'twister');
    agent = rlPPOAgent(oi, ai); %#ok<NASGU>
    agent.AgentOptions.SampleTime = 0.025;
    files{k} = fullfile(outDir, sprintf('D6_untrained_ppo_seed%d.mat', k));
    save(files{k}, 'agent');
end

jobs = {};
for k = 1:5
    for e = 1:nPerAgent
        jobs(end + 1, :) = {'untrained', k, desktop_config('agentFile', files{k}, 'agentLabel', sprintf('untrained%d', k), ...
            'explore', true, 'seed', 1000 * k + e, 'keepTs', true)}; %#ok<AGROW>
    end
end
rng(6, 'twister');
mf = max(0.5, 1 + 0.65 * randn(nCdr, 1));
dl = randi([1 3], nCdr, 1);
for e = 1:nCdr
    jobs(end + 1, :) = {'cdr_p4', 0, desktop_config('agentFile', sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
        'agentLabel', 'CDR2-4', 'explore', true, 'seed', e, 'base_mass', 65 * mf(e), 'delay_steps', dl(e), 'keepTs', true)}; %#ok<AGROW>
end
key = cellfun(@(c) c.base_mass, jobs(:, 3));
[~, order] = sort(key);
jobs = jobs(order, :);

n = size(jobs, 1);
fprintf('D6: %d Episoden\n', n);
rows = cell(n, 1);
rew = cell(n, 1);
tAll = tic;
for i = 1:n
    c = jobs{i, 3};
    r = struct('set', jobs{i, 1}, 'agent', c.agentLabel, 'seed', c.seed, 'base_mass', c.base_mass, ...
        'delay_steps', c.delay_steps, 'stop_reason', 'error', 't_end', NaN, 'n_steps', NaN, 'max_ee', NaN, ...
        'rew_step_median', NaN, 'rew_step_min', NaN, 'sim_error', '');
    try
        res = desktop_run_episode(c);
        m = res.metrics;
        rr = res.ts.reward(:);
        if m.early_stop, rr = rr(1:end - 1); end      % letzter Schritt ist die -50 des Abbruchs
        r.stop_reason = m.stop_reason; r.t_end = m.t_end; r.n_steps = m.n_steps; r.max_ee = m.max_ee;
        r.rew_step_median = median(rr); r.rew_step_min = min(rr);
        rew{i} = rr;
    catch err
        r.sim_error = strtok(err.message, newline);   % z. B. Solverabbruch, falls Simscape selbst scheitert
    end
    rows{i} = r;
    if mod(i, 50) == 0 || i == n, fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll)); end
end
E = struct2table([rows{:}]);

% Zusammenfassung
R = struct();
for s = {'untrained', 'cdr_p4'}
    idx = strcmp(E.set, s{1});
    sub = E(idx, :);
    rr = vertcat(rew{idx});
    x = struct('n', height(sub), ...
        'stop_none', sum(strcmp(sub.stop_reason, 'none')), 'stop_nonfinite', sum(strcmp(sub.stop_reason, 'nonfinite')), ...
        'stop_ep', sum(strcmp(sub.stop_reason, 'ep>0.5m')), 'stop_ev', sum(strcmp(sub.stop_reason, 'ev>2m/s')), ...
        'stop_ori', sum(strcmp(sub.stop_reason, 'ori>1rad')), 'stop_other', sum(strcmp(sub.stop_reason, 'unknown')), ...
        'sim_errors', sum(~cellfun(@isempty, sub.sim_error)), 't_end_median', median(sub.t_end, 'omitnan'), ...
        'rew_step_median', median(rr), 'rew_step_p10', prctile(rr, 10), 'rew_step_p90', prctile(rr, 90), ...
        'rew_step_min', min(rr), 'share_below_m13', mean(rr < -13));
    R.(s{1}) = x;
    fprintf('\n%s:\n', s{1}); disp(x);
end

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if nPerAgent < 40, stamp = [stamp '_test']; end
save(fullfile(outDir, ['D6_divergence_' stamp '.mat']), 'E', 'R', 'rew', 'jobs', '-v7.3');
writetable(E, fullfile(outDir, ['D6_divergence_' stamp '_episodes.csv']));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D6_divergence_' stamp '.*']), toc(tAll) / 60);
end

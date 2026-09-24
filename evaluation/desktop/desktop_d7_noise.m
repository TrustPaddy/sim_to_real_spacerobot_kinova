function [E, S] = desktop_d7_noise(nSeeds)
%DESKTOP_D7_NOISE  Robustheit der Tracking-Agenten gegen Beobachtungsrauschen (Desktop-Plan D7, R1.7).
%   [E, S] = desktop_d7_noise()     10 Seeds je Stufe
%   [E, S] = desktop_d7_noise(2)    Kurztest
%
%   CDR2-4 (40 Hz) und ppo_10hz (10 Hz), deterministische Policy, SK_desktop mit 65 kg, Halbkreis 8,5 s.
%   Gaussches Rauschen auf der Beobachtung vor dem Agenten, Standardabweichung = Stufe x sigma_nom. sigma_nom sind
%   die Rauschstaerken aus training/SpaceKinova_Point_CDR.m (Phase 4): EE-Fehler 3 mm, EE-Geschwindigkeit 10 mm/s,
%   Basisgeschwindigkeiten 5 mm/s und 5 mrad/s, Gelenkwinkel 2 mrad, Gelenkgeschwindigkeiten 10 mrad/s,
%   Orientierungsfehler 3 mrad. Keiner der beiden Tracking-Agenten wurde mit Rauschen trainiert.
%     all_x     alle Gruppen, Stufe 1, 2, 5, 10
%     <gruppe>  nur eine Gruppe, Stufe 5 (ep, ev, q, dq, base)
%   Ergebnis: data/simulation/desktop/D7_noise_<Zeit>.mat, _episodes.csv, _summary.csv

if nargin < 1, nSeeds = 10; end
setup_project;
desktop_build_model();

sig = [0.003 * ones(3,1); 0.010 * ones(3,1); 0.005 * ones(3,1); 0.005 * ones(3,1); ...
       0.002 * ones(7,1); 0.010 * ones(7,1); 0.003 * ones(3,1)];
grp = struct('ep', 1:3, 'ev', 4:6, 'base', [7:12 27:29], 'q', 13:19, 'dq', 20:26);

ag = struct('label', {'CDR2-4', 'ppo_10hz'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'ppo_10hz.mat')}, 'Ts', {0.025, 0.1});

jobs = {};
for a = 1:numel(ag)
    base = {'agentFile', ag(a).file, 'agentLabel', ag(a).label, 'Ts_agent', ag(a).Ts, 'keepTs', false};
    jobs(end + 1, :) = {'none', 0, 0, desktop_config(base{:})}; %#ok<AGROW>
    for lvl = [1 2 5 10]
        for k = 1:nSeeds
            jobs(end + 1, :) = {'all', lvl, k, desktop_config(base{:}, 'obs_noise', lvl * sig, 'seed', k)}; %#ok<AGROW>
        end
    end
    for g = fieldnames(grp).'
        s = zeros(29, 1); s(grp.(g{1})) = 5 * sig(grp.(g{1}));
        for k = 1:nSeeds
            jobs(end + 1, :) = {g{1}, 5, k, desktop_config(base{:}, 'obs_noise', s, 'seed', k)}; %#ok<AGROW>
        end
    end
end

n = size(jobs, 1);
fprintf('D7: %d Episoden\n', n);
rows = cell(n, 1);
tAll = tic;
for i = 1:n
    c = jobs{i, 4};
    r = struct('noise', jobs{i, 1}, 'level', jobs{i, 2}, 'seed', jobs{i, 3}, 'agent', c.agentLabel, ...
        'mse_ee', NaN, 'rms_ee', NaN, 'max_ee', NaN, 'completed', false, 'ood04', NaN, 'stop_reason', 'error');
    try
        m = desktop_run_episode(c).metrics;
        r.mse_ee = m.mse_ee; r.rms_ee = m.rms_ee; r.max_ee = m.max_ee; r.completed = ~m.early_stop;
        r.ood04 = m.ood04; r.stop_reason = m.stop_reason;
    catch err
        warning('D7:episode', '%s fehlgeschlagen: %s', c.agentLabel, err.message);
    end
    rows{i} = r;
    if mod(i, 50) == 0 || i == n, fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll)); end
end
E = struct2table([rows{:}]);

G = findgroups(E.agent, E.noise, E.level);
S = table();
for g = unique(G).'
    sub = E(G == g, :);
    S = [S; table(sub.agent(1), sub.noise(1), sub.level(1), height(sub), mean(sub.mse_ee), std(sub.mse_ee), ...
        mean(sub.max_ee), max(sub.max_ee), mean(sub.completed), mean(sub.ood04), ...
        'VariableNames', {'agent', 'noise', 'level', 'n', 'mse_mean', 'mse_std', 'max_mean', 'max_max', ...
        'completion', 'ood04_share'})]; %#ok<AGROW>
end
disp(S);

outDir = sk_path('data', 'simulation', 'desktop');
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if nSeeds < 10, stamp = [stamp '_test']; end
save(fullfile(outDir, ['D7_noise_' stamp '.mat']), 'E', 'S', 'jobs', 'sig', '-v7.3');
writetable(E, fullfile(outDir, ['D7_noise_' stamp '_episodes.csv']));
writetable(S, fullfile(outDir, ['D7_noise_' stamp '_summary.csv']));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D7_noise_' stamp '.*']), toc(tAll) / 60);
end

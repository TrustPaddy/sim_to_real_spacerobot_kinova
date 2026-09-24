function [T, S] = desktop_d4_setpoint(nSeeds)
%DESKTOP_D4_SETPOINT  Set-Point-Agenten in Simulation ueber die Startliste der Laborreihe (Desktop-Plan D4).
%   [T, S] = desktop_d4_setpoint()     volle Reihe (20 stochastische Seeds fuer den alten OOD-Start)
%   [T, S] = desktop_d4_setpoint(2)    Kurztest
%
%   Pruefung: SK_point gegen das Originalmodell, SK_point_fixed auf Wiederholbarkeit (je eine Episode).
%   Starts: S00-S14 aus hardware/campaign/setpoint_starts.csv (Ziel [0,479, -0,005, 1,136] m, 25 s) und der
%   alte OOD-Start [0 -90 0 0 0 0 0] Grad von Table VII. Deterministisch, ausser wo angegeben.
%   Saetze:
%     fixed1_train     test_agent_fixed1 in seinem Trainingsmodell (feste Basis, Schwerkraft), ohne Rauschen
%     fixed1_noise     wie oben mit dem Rauschen der Trainingsphase 4 (Seed = Startnummer)
%     fixed1_free65    test_agent_fixed1 im frei schwebenden Modell, 65 kg
%     rand2_free65     test_agent_rand2 frei schwebend, 65 kg
%     rand2_free650    test_agent_rand2 frei schwebend, 650 kg (Stand der bisherigen Table VII)
%     tab7_*           alter OOD-Start, rand2 mit 650 kg und 65 kg, deterministisch und nSeeds stochastisch
%   Ergebnis: data/simulation/desktop/D4_setpoint_<Zeit>.mat, _episodes.csv, _summary.csv

if nargin < 1, nSeeds = 20; end
setup_project;
desktop_build_point_models();

aFixed = sk_path('SavedAgents', 'MotionProfile', 'point', 'test_agent_fixed1.mat');
aRand = sk_path('SavedAgents', 'MotionProfile', 'point', 'test_agent_rand2.mat');
qOOD = [0 -90 0 0 0 0 0];

% --- Pruefung gegen die Originale ---
chk = {};
c1 = struct('agentFile', aFixed, 'agentLabel', 'fixed1', 'q0_deg', [0 15 180 -130 0 55 90]);
% Das Original _point_fixed laeuft mit sim() nicht (Rauschblock mit kontinuierlicher Abtastzeit), deshalb
% nur Wiederholbarkeit von SK_point_fixed ohne Rauschen.
chk(end + 1, :) = {'SK_point_fixed_a', desktop_run_setpoint(setfield(c1, 'model', 'SK_point_fixed'))}; %#ok<SFLD>
chk(end + 1, :) = {'SK_point_fixed_b', desktop_run_setpoint(setfield(c1, 'model', 'SK_point_fixed'))}; %#ok<SFLD>
c2 = struct('agentFile', aRand, 'agentLabel', 'rand2', 'q0_deg', qOOD, 'base_mass', 650);
chk(end + 1, :) = {'orig_point', desktop_run_setpoint(setfield(c2, 'model', 'SpaceKinova_MotionProfile_point'))}; %#ok<SFLD>
chk(end + 1, :) = {'SK_point', desktop_run_setpoint(setfield(c2, 'model', 'SK_point'))}; %#ok<SFLD>
for k = [1 3]
    a = chk{k, 2}; b = chk{k + 1, 2};
    n = min(numel(a.ts.epn), numel(b.ts.epn));
    fprintf('Pruefung %s gegen %s: max|d epn| = %.2e m, Endfehler %.4f / %.4f m\n', chk{k, 1}, chk{k + 1, 1}, ...
        max(abs(a.ts.epn(1:n) - b.ts.epn(1:n))), a.metrics.final_err, b.metrics.final_err);
end

% --- Starts ---
St = readtable(sk_path('hardware', 'campaign', 'setpoint_starts.csv'), 'CommentStyle', '#');
starts = struct('id', {}, 'q', {}, 'level', {});
for i = 1:height(St)
    starts(end + 1) = struct('id', St.id{i}, 'q', [St.q1_deg(i) St.q2_deg(i) St.q3_deg(i) St.q4_deg(i) ...
        St.q5_deg(i) St.q6_deg(i) St.q7_deg(i)], 'level', St.level_m(i)); %#ok<AGROW>
end
starts(end + 1) = struct('id', 'OOD', 'q', qOOD, 'level', NaN);

jobs = {};
for s = 1:numel(starts)
    b = struct('q0_deg', starts(s).q, 'label', starts(s).id);
    jobs(end + 1, :) = {'fixed1_train', starts(s), mk(b, 'SK_point_fixed', aFixed, 'fixed1', 65, 0, false, 0)}; %#ok<AGROW>
    jobs(end + 1, :) = {'fixed1_noise', starts(s), mk(b, 'SK_point_fixed', aFixed, 'fixed1', 65, 1, false, s)}; %#ok<AGROW>
    jobs(end + 1, :) = {'fixed1_free65', starts(s), mk(b, 'SK_point', aFixed, 'fixed1', 65, 0, false, 0)}; %#ok<AGROW>
    jobs(end + 1, :) = {'rand2_free65', starts(s), mk(b, 'SK_point', aRand, 'rand2', 65, 0, false, 0)}; %#ok<AGROW>
    jobs(end + 1, :) = {'rand2_free650', starts(s), mk(b, 'SK_point', aRand, 'rand2', 650, 0, false, 0)}; %#ok<AGROW>
end
b = struct('q0_deg', qOOD, 'label', 'OOD');
for mass = [650 65]
    for k = 1:nSeeds
        jobs(end + 1, :) = {sprintf('tab7_stoch_%dkg', mass), starts(end), ...
            mk(b, 'SK_point', aRand, 'rand2', mass, 0, true, k)}; %#ok<AGROW>
    end
end

% nach Modell und Masse sortieren
key = cellfun(@(c) sprintf('%s_%06.0f', c.model, c.base_mass), jobs(:, 3), 'UniformOutput', false);
[~, order] = sort(key);
jobs = jobs(order, :);

n = size(jobs, 1);
fprintf('D4: %d Episoden\n', n);
res = cell(n, 1);
tAll = tic;
for i = 1:n
    try
        c = jobs{i, 3};
        c.keepTs = ~startsWith(jobs{i, 1}, 'tab7');
        res{i} = desktop_run_setpoint(c);
    catch err
        warning('D4:episode', '%s %s fehlgeschlagen: %s', jobs{i, 1}, jobs{i, 2}.id, err.message);
        res{i} = struct('cfg', jobs{i, 3}, 'metrics', struct(), 'ts', struct(), 'info', struct('error', err.message));
    end
    if mod(i, 20) == 0 || i == n, fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll)); end
end

rows = cell(n, 1);
for i = 1:n
    m = res{i}.metrics; c = jobs{i, 3}; st = jobs{i, 2};
    r = struct('set', jobs{i, 1}, 'start', st.id, 'level_m', st.level, 'agent', c.agentLabel, ...
        'model', c.model, 'base_mass', c.base_mass, 'noise', c.noise_scale, 'explore', c.explore, 'seed', c.seed);
    f = {'d0', 'final_err', 'min_err', 't_end', 'converged', 'failed', 'success50', 'settle_t', ...
        'frac_softzone_J2', 'frac_softzone_J4', 'frac_softzone_J6', 'qmax_deg_J2', 'qmax_deg_J4', ...
        'qmax_deg_J6', 'frac_atlim', 'frac_beyond_hw', 'ori_max', 'ret'};
    for k = 1:numel(f)
        if isfield(m, f{k}), r.(f{k}) = m.(f{k}); else, r.(f{k}) = NaN; end
    end
    rows{i} = r;
end
T = struct2table([rows{:}]);

% Zusammenfassung je Satz (ohne OOD-Start bei den Startlisten-Saetzen)
S = table();
for s = unique(T.set).'
    sub = T(strcmp(T.set, s{1}), :);
    if ~startsWith(s{1}, 'tab7'), sub = sub(~strcmp(sub.start, 'OOD'), :); end
    row = table(s, height(sub), mean(sub.success50), mean(sub.converged), mean(sub.failed), ...
        median(sub.final_err), max(sub.final_err), mean(sub.settle_t, 'omitnan'), ...
        mean(sub.frac_softzone_J6 > 0.5), mean(sub.frac_beyond_hw > 0), ...
        'VariableNames', {'set', 'n', 'success50', 'converged', 'failed', 'final_err_median', ...
        'final_err_max', 'settle_t_mean', 'share_J6_softzone_gt50pct', 'share_beyond_hw'});
    S = [S; row]; %#ok<AGROW>
end
disp(S);
disp(T(~startsWith(T.set, 'tab7'), {'set', 'start', 'level_m', 'd0', 'final_err', 'converged', 'success50', ...
    't_end', 'frac_softzone_J6', 'qmax_deg_J6', 'qmax_deg_J4', 'frac_beyond_hw'}));

outDir = sk_path('data', 'simulation', 'desktop');
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if nSeeds < 20, stamp = [stamp '_test']; end
save(fullfile(outDir, ['D4_setpoint_' stamp '.mat']), 'T', 'S', 'res', 'jobs', 'chk', '-v7.3');
writetable(T, fullfile(outDir, ['D4_setpoint_' stamp '_episodes.csv']));
writetable(S, fullfile(outDir, ['D4_setpoint_' stamp '_summary.csv']));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D4_setpoint_' stamp '.*']), toc(tAll) / 60);
end

function c = mk(b, model, agentFile, agentLabel, mass, noise, explore, seed)
c = b;
c.model = model; c.agentFile = agentFile; c.agentLabel = agentLabel; c.base_mass = mass;
c.noise_scale = noise; c.explore = explore; c.seed = seed;
end

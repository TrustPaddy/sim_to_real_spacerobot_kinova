function out = desktop_d2_timing(parts)
%DESKTOP_D2_TIMING  Ratenreihe, Referenztakt und Nachbau der V2.1-Hardwarelaeufe (Desktop-Plan D2).
%   out = desktop_d2_timing()            Teile A bis E
%   out = desktop_d2_timing('B')         nur Teil B
%
%   Befund vorab (Logs der Laeufe 008-014): In V2.1 rueckte die Referenz pro Schritt um 25 ms vor, ein
%   Schritt dauerte auf der Hardware aber etwa 130-160 ms (aus Gelenkweg / Geschwindigkeit geschaetzt).
%   Der Rate Limiter rechnete ebenfalls fest mit 25 ms. Dazu kommt der Beobachtungsfehler A22.
%
%   A  Faktorversuch auf fester Basis (1e9 kg, wie im Labor), deterministisch, Agenten PPO_base und CDR2-4:
%      Rate {40 Hz, 7,1 Hz (140 ms)} x Referenz {Wandzeit, pro Schritt 25 ms} x Beobachtung {Training, V2.1}
%      x Skalierung {1,0; 0,5} x Rate Limiter {Training 0,5/s, V2.1 fest 0,0125 pro Schritt}
%      (bei 40 Hz sind beide Rate-Limiter-Varianten gleich)
%   B  Nachbau der V2.1-Laeufe 008-011 (PPO_base) und 012-014 (CDR2-4) mit ihrem Faktor, alle V2.1-Eigenheiten,
%      Schrittdauer 100, 125 und 150 ms. Vergleich mit dem Log: Schritte bis zum OOD-Stopp (0,4 m) und
%      EE-Fehler pro Schritt
%   C  Ratenreihe auf frei schwebender Basis (65 kg), korrekte Beobachtung, vier Agenten,
%      Schrittdauer 25, 50, 75, 100, 125, 140, 175 ms, Referenz nach Wandzeit und pro Schritt um die
%      Trainingsschrittweite (Referenz verlangsamt um Ts_train / Ts)
%   D  Nachbau der V2.2-Laeufe 012 (Faktor 0,6) und 013 (Faktor 1,0) mit ppo_10hz, 110 ms pro Schritt,
%      Referenz nach Wandzeit, mit und ohne Beobachtungsfehler, dazu Faktor 0,5 / 0,7 / 1,0
%   E  Vorhersage fuer die Laborreihe (V2.4): korrekte Beobachtung, Wandzeit, Faktor 1,0, Rate Limiter mit der
%      Trainingsschrittweite pro Schritt, feste Basis, Schritt 100 / 125 / 140 / 175 ms
%   Ergebnis: data/simulation/desktop/D2_timing_<Zeit>.mat und _A bis _E.csv

if nargin < 1, parts = 'ABCDE'; end
setup_project;
desktop_build_model();

ag = struct( ...
    'label', {'CDR2-4', 'Optimized', 'PPO_base', 'ppo_10hz'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'SpaceKinova_PPO_agent_motionprofile.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'ppo_10hz.mat')}, ...
    'trainTs', {0.025, 0.025, 0.025, 0.1});
agFile = containers.Map({ag.label}, {ag.file});
agTs = containers.Map({ag.label}, {ag.trainTs});

jobs = {};   % {Teil, Info-Struct, cfg}

% ---------------- Teil A ----------------
if contains(parts, 'A')
    for a = {'PPO_base', 'CDR2-4'}
        for Tsr = [0.025 0.14]
            for ref = {'wall', 'sample'}
                for om = [0 1]
                    for sc = [1.0 0.5]
                        slews = {'train'};
                        if Tsr > 0.025, slews = {'train', 'v21'}; end
                        for sl = slews
                            slew = 0.5;
                            if strcmp(sl{1}, 'v21'), slew = 0.5 * 0.025 / Tsr; end
                            c = desktop_config('agentFile', agFile(a{1}), 'agentLabel', a{1}, 'Ts_agent', Tsr, ...
                                'base_mass', 1e9, 'ref_timing', ref{1}, 'Ts_ref_step', 0.025, 'obs_mode', om, ...
                                'cmd_scale', sc, 'slew', slew, 'T', 8.5, 'keepTs', true);
                            info = struct('rate', sprintf('%gms', 1000 * Tsr), 'ref', ref{1}, 'obs', om, ...
                                'scale', sc, 'slew', sl{1}, 'hwrun', '', 'hw_steps', NaN);
                            jobs(end + 1, :) = {'A', info, c}; %#ok<AGROW>
                        end
                    end
                end
            end
        end
    end
end

% ---------------- Teil B ----------------
hw = struct('run', {'run_008_agent_train_seed1_singular', 'run_009_agent_train_seed2_singular', ...
    'run_010_agent_train_seed3_singular', 'run_011_agent_train_seed4_singular', ...
    'run_012_agent_train_seed1_cdr', 'run_013_agent_train_seed2_cdr', 'run_014_agent_train_seed3_cdr'}, ...
    'agent', {'PPO_base', 'PPO_base', 'PPO_base', 'PPO_base', 'CDR2-4', 'CDR2-4', 'CDR2-4'});
hwData = struct();
if contains(parts, 'B')
    for h = 1:numel(hw)
        S = load(sk_path('data', 'hardware', 'deploy_logs', [hw(h).run '.mat']));
        n = S.meta.kEnd;
        hwData.(strrep(hw(h).run, '-', '_')) = struct('ep_norm', S.data.ep_norm(1:n), 'kEnd', n, ...
            'scale', S.meta.speedScale);
        for Tsr = [0.10 0.125 0.15]
            c = desktop_config('agentFile', agFile(hw(h).agent), 'agentLabel', hw(h).agent, 'Ts_agent', Tsr, ...
                'base_mass', 1e9, 'ref_timing', 'sample', 'Ts_ref_step', 0.025, 'obs_mode', 1, ...
                'cmd_scale', S.meta.speedScale, 'slew', 0.5 * 0.025 / Tsr, 'T', 12, 'T_path', 8.5, 'keepTs', true);
            info = struct('rate', sprintf('%gms', 1000 * Tsr), 'ref', 'sample', 'obs', 1, ...
                'scale', S.meta.speedScale, 'slew', 'v21', 'hwrun', hw(h).run, 'hw_steps', n);
            jobs(end + 1, :) = {'B', info, c}; %#ok<AGROW>
        end
    end
end

% ---------------- Teil D ----------------
% Nachbau der V2.2-Laeufe mit ppo_10hz (gleicher Beobachtungsfehler, Referenz nach Wandzeit, Rate Limiter mit
% 0,1 s pro Schritt wie im Training). Schritt 110 ms (Lauf 012: Mittel 116 ms, Median 112 ms).
hw22 = struct('run', {'run_012_agent_train_seed10_okay', 'run_013_agent_train_seed5'});
if contains(parts, 'D')
    for h = 1:numel(hw22)
        S = load(sk_path('data', 'hardware', 'deploy_logs', [hw22(h).run '.mat']));
        n = S.meta.kEnd;
        hwData.(hw22(h).run) = struct('ep_norm', S.data.ep_norm(1:n), 'kEnd', n, 'scale', S.meta.speedScale);
        for om = [1 0]
            c = desktop_config('agentFile', agFile('ppo_10hz'), 'agentLabel', 'ppo_10hz', 'Ts_agent', 0.11, ...
                'base_mass', 1e9, 'ref_timing', 'wall', 'obs_mode', om, 'cmd_scale', S.meta.speedScale, ...
                'T', 8.5, 'keepTs', true);
            info = struct('rate', '110ms', 'ref', 'wall', 'obs', om, 'scale', S.meta.speedScale, ...
                'slew', 'train', 'hwrun', hw22(h).run, 'hw_steps', n);
            jobs(end + 1, :) = {'D', info, c}; %#ok<AGROW>
        end
    end
    for sc = [0.5 0.7 1.0]
        c = desktop_config('agentFile', agFile('ppo_10hz'), 'agentLabel', 'ppo_10hz', 'Ts_agent', 0.11, ...
            'base_mass', 1e9, 'ref_timing', 'wall', 'obs_mode', 1, 'cmd_scale', sc, 'T', 8.5, 'keepTs', true);
        info = struct('rate', '110ms', 'ref', 'wall', 'obs', 1, 'scale', sc, 'slew', 'train', ...
            'hwrun', '', 'hw_steps', NaN);
        jobs(end + 1, :) = {'D', info, c}; %#ok<AGROW>
    end
end

% ---------------- Teil E ----------------
% Vorhersage fuer die Laborreihe (deploy_tracking_v24): korrekte Beobachtung, Referenz nach Wandzeit,
% Faktor 1,0, Rate Limiter mit der Trainingsschrittweite des Agenten pro Schritt (cfg.pipelineTs), feste Basis.
% Die erreichbare Schrittdauer ist offen, deshalb mehrere Werte.
if contains(parts, 'E')
    for a = {'CDR2-4', 'PPO_base', 'ppo_10hz'}
        for Tsr = [0.10 0.125 0.14 0.175]
            c = desktop_config('agentFile', agFile(a{1}), 'agentLabel', a{1}, 'Ts_agent', Tsr, ...
                'base_mass', 1e9, 'ref_timing', 'wall', 'obs_mode', 0, 'cmd_scale', 1, ...
                'slew', 0.5 * agTs(a{1}) / Tsr, 'T', 8.5, 'keepTs', true);
            info = struct('rate', sprintf('%gms', 1000 * Tsr), 'ref', 'wall', 'obs', 0, 'scale', 1, ...
                'slew', 'v24', 'hwrun', '', 'hw_steps', NaN);
            jobs(end + 1, :) = {'E', info, c}; %#ok<AGROW>
        end
    end
end

% ---------------- Teil C ----------------
if contains(parts, 'C')
    for a = 1:numel(ag)
        for Tsr = [0.025 0.05 0.075 0.1 0.125 0.14 0.175]
            for ref = {'wall', 'sample'}
                c = desktop_config('agentFile', ag(a).file, 'agentLabel', ag(a).label, 'Ts_agent', Tsr, ...
                    'base_mass', 65, 'ref_timing', ref{1}, 'Ts_ref_step', ag(a).trainTs, 'T', 8.5, 'keepTs', false);
                info = struct('rate', sprintf('%gms', 1000 * Tsr), 'ref', ref{1}, 'obs', 0, ...
                    'scale', 1, 'slew', 'train', 'hwrun', '', 'hw_steps', NaN);
                jobs(end + 1, :) = {'C', info, c}; %#ok<AGROW>
            end
        end
    end
end

% Nach Masse sortieren (weniger Neukompilierungen)
masses = cellfun(@(c) c.base_mass, jobs(:, 3));
[~, order] = sort(masses);
jobs = jobs(order, :);

n = size(jobs, 1);
fprintf('D2: %d Episoden\n', n);
res = cell(n, 1);
tAll = tic;
for i = 1:n
    c = jobs{i, 3};
    c.label = sprintf('%s_%d', jobs{i, 1}, i);
    try
        res{i} = desktop_run_episode(c);
    catch err
        warning('D2:episode', '%s fehlgeschlagen: %s', c.label, err.message);
        res{i} = struct('cfg', c, 'metrics', struct(), 'ts', struct(), 'info', struct('error', err.message));
    end
    if mod(i, 20) == 0 || i == n
        fprintf('  %d/%d (%.0f s)\n', i, n, toc(tAll));
    end
end

% ---------------- Tabelle ----------------
rows = cell(n, 1);
for i = 1:n
    c = res{i}.cfg; m = res{i}.metrics; inf_ = jobs{i, 2};
    r = struct('part', jobs{i, 1}, 'agent', c.agentLabel, 'train_ms', 1000 * agTs(c.agentLabel), ...
        'step_ms', round(1000 * c.Ts_agent), 'base', ternary(c.base_mass > 1e6, 'fixed', 'free'), ...
        'ref', inf_.ref, 'obs_v21', inf_.obs, 'scale', inf_.scale, 'slew', inf_.slew, ...
        'hwrun', inf_.hwrun, 'hw_steps', inf_.hw_steps, ...
        'ood04', NaN, 'ood04_step', NaN, 'ood04_t', NaN, 'mse_ee', NaN, 'max_ee', NaN, 'completed', NaN, ...
        'stop_reason', 'error', 'n_steps', NaN, 'ep_step_rmsdiff_hw', NaN);
    if isfield(m, 'mse_ee')
        r.ood04 = m.ood04; r.ood04_step = m.ood04_step; r.ood04_t = m.ood04_t;
        r.mse_ee = m.mse_ee; r.max_ee = m.max_ee; r.completed = ~m.early_stop;
        r.stop_reason = m.stop_reason; r.n_steps = m.n_steps;
        if ~isempty(inf_.hwrun)
            hd = hwData.(strrep(inf_.hwrun, '-', '_'));
            epS = interp1(res{i}.ts.t_ee, vecnorm(res{i}.ts.ep, 2, 2), res{i}.ts.t_agent, 'previous', 'extrap');
            k = min(numel(epS), numel(hd.ep_norm) - 1);
            r.ep_step_rmsdiff_hw = sqrt(mean((epS(1:k) - hd.ep_norm(1:k)).^2));
        end
    end
    rows{i} = r;
end
T = struct2table([rows{:}]);

outDir = sk_path('data', 'simulation', 'desktop');
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out = struct('T', T, 'res', {res}, 'jobs', {jobs}, 'hwData', hwData);
save(fullfile(outDir, ['D2_timing_' stamp '.mat']), '-struct', 'out', '-v7.3');
for p = 'ABCDE'
    if any(strcmp(T.part, p))
        writetable(T(strcmp(T.part, p), :), fullfile(outDir, sprintf('D2_timing_%s_%s.csv', stamp, p)));
    end
end
disp(T(:, {'part', 'agent', 'step_ms', 'base', 'ref', 'obs_v21', 'scale', 'slew', 'hwrun', 'hw_steps', ...
    'ood04', 'ood04_step', 'mse_ee', 'max_ee', 'ep_step_rmsdiff_hw'}));
fprintf('Gespeichert: %s (%.1f min)\n', fullfile(outDir, ['D2_timing_' stamp '.*']), toc(tAll) / 60);
end

function out = ternary(cond, a, b)
if cond, out = a; else, out = b; end
end

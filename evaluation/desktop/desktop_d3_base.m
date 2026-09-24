function out = desktop_d3_base()
%DESKTOP_D3_BASE  Frei schwebende gegen fest montierte Basis in Simulation (Desktop-Plan D3, AE.2, R2.2).
%   out = desktop_d3_base()
%
%   A  Vier Tracking-Agenten, deterministisch, Halbkreis 8,5 s, korrekte Beobachtung, Referenz nach Wandzeit,
%      jeweils bei 40 Hz und 10 Hz, Basis frei (65 kg) und fest (1e9 kg). Kennzahlen: EE-Fehler, Basisdrehung,
%      Basisverschiebung und Unterschied der Gelenkbefehle zwischen frei und fest.
%   B  Zerlegung der Luecke an den Hardwarelaeufen 074-078 (V2.3: ppo_10hz, korrekte Beobachtung, Bahn 17 s,
%      Faktor 0,35, Referenz nach Wandzeit, Rate Limiter mit 0,1 s pro Schritt, 164-179 ms pro Schritt):
%      Simulation frei -> Simulation fest -> Hardware. Schrittdauer 165 und 175 ms.
%   Ergebnis: data/simulation/desktop/D3_base_<Zeit>.mat, _A.csv, _B.csv

setup_project;
desktop_build_model();

ag = struct( ...
    'label', {'CDR2-4', 'Optimized', 'PPO_base', 'ppo_10hz'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'SpaceKinova_PPO_agent_motionprofile.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'ppo_10hz.mat')});

% ---------------- A ----------------
resA = {};
rowsA = {};
for a = 1:numel(ag)
    for Tsr = [0.025 0.1]
        r = struct();
        for base = {'free', 'fixed'}
            mass = 65; if strcmp(base{1}, 'fixed'), mass = 1e9; end
            c = desktop_config('agentFile', ag(a).file, 'agentLabel', ag(a).label, 'Ts_agent', Tsr, 'base_mass', mass);
            r.(base{1}) = desktop_run_episode(c);
        end
        resA(end + 1, :) = {ag(a).label, Tsr, r}; %#ok<AGROW>
        mf = r.free.metrics; mx = r.fixed.metrics;
        n = min(size(r.free.ts.a_raw, 1), size(r.fixed.ts.a_raw, 1));
        da = r.free.ts.a_raw(1:n, :) - r.fixed.ts.a_raw(1:n, :);
        rowsA{end + 1} = struct('agent', ag(a).label, 'rate_Hz', round(1 / Tsr), ...
            'mse_free', mf.mse_ee, 'mse_fixed', mx.mse_ee, 'mse_change', mx.mse_ee / mf.mse_ee - 1, ...
            'max_free', mf.max_ee, 'max_fixed', mx.max_ee, ...
            'ori_max_free', mf.ori_max, 'w_mean_free', mf.w_mean, 'base_disp_free', mf.base_disp_max, ...
            'ori_max_fixed', mx.ori_max, 'a_raw_rms_diff', sqrt(mean(da(:).^2)), ...
            'completed_free', ~mf.early_stop, 'completed_fixed', ~mx.early_stop); %#ok<AGROW>
    end
end
TA = struct2table([rowsA{:}]);
disp(TA);

% ---------------- B ----------------
runs = arrayfun(@(k) sprintf('run_%03d_agent_train_seed11', k), 74:78, 'UniformOutput', false);
hw = struct('run', {}, 't', {}, 'epn', {}, 'rms', {}, 'max', {}, 'dt_mean', {});
for k = 1:numel(runs)
    S = load(sk_path('data', 'hardware', 'runs', [runs{k} '.mat']));
    n = S.meta.kEnd;
    e = vecnorm(S.data.ee_measured(1:n, :) - S.data.ee_ref(1:n, :), 2, 2);
    hw(k) = struct('run', runs{k}, 't', S.data.t_ref(1:n), 'epn', e, 'rms', sqrt(mean(e.^2)), 'max', max(e), ...
        'dt_mean', mean(S.data.dt_loop(2:n), 'omitnan'));
end
fprintf('Hardware 074-078: RMS %.4f +- %.4f m, max %.4f m, Schritt %.0f ms\n', mean([hw.rms]), std([hw.rms]), ...
    mean([hw.max]), 1000 * mean([hw.dt_mean]));

resB = {};
rowsB = {};
a10 = ag(4).file;
for Tsr = [0.165 0.175]
    for base = {'free', 'fixed'}
        mass = 65; if strcmp(base{1}, 'fixed'), mass = 1e9; end
        c = desktop_config('agentFile', a10, 'agentLabel', 'ppo_10hz', 'Ts_agent', Tsr, 'base_mass', mass, ...
            'T', 17, 'T_path', 17, 'cmd_scale', 0.35, 'slew', 0.5 * 0.1 / Tsr, 'ref_timing', 'wall');
        r = desktop_run_episode(c);
        resB(end + 1, :) = {base{1}, Tsr, r}; %#ok<AGROW>
        epS = vecnorm(r.ts.ep, 2, 2);
        dd = zeros(numel(hw), 1);
        for k = 1:numel(hw)
            s = interp1(r.ts.t_ee, epS, min(hw(k).t, r.ts.t_ee(end)), 'linear');
            dd(k) = sqrt(mean((s - hw(k).epn).^2));
        end
        rowsB{end + 1} = struct('level', ['sim_' base{1}], 'step_ms', round(1000 * Tsr), ...
            'rms_ee', r.metrics.rms_ee, 'max_ee', r.metrics.max_ee, 'ori_max', r.metrics.ori_max, ...
            'rms_diff_to_hw', mean(dd)); %#ok<AGROW>
    end
end
rowsB{end + 1} = struct('level', 'hardware_074_078', 'step_ms', round(1000 * mean([hw.dt_mean])), ...
    'rms_ee', mean([hw.rms]), 'max_ee', mean([hw.max]), 'ori_max', NaN, 'rms_diff_to_hw', NaN);
TB = struct2table([rowsB{:}]);
disp(TB);

outDir = sk_path('data', 'simulation', 'desktop');
stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
out = struct('TA', TA, 'TB', TB, 'resA', {resA}, 'resB', {resB}, 'hw', hw);
save(fullfile(outDir, ['D3_base_' stamp '.mat']), '-struct', 'out', '-v7.3');
writetable(TA, fullfile(outDir, ['D3_base_' stamp '_A.csv']));
writetable(TB, fullfile(outDir, ['D3_base_' stamp '_B.csv']));
fprintf('Gespeichert: %s\n', fullfile(outDir, ['D3_base_' stamp '.*']));
end

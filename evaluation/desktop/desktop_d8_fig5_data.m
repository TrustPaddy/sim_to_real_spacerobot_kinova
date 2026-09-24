function desktop_d8_fig5_data(nEp)
%DESKTOP_D8_FIG5_DATA  Daten fuer Fig. 5 (fig:ppo40hz) neu erzeugen (Desktop-Plan D8, R1.4, Befund A51).
%   desktop_d8_fig5_data()      50 stochastische Episoden je Agent (wie die alte Caption)
%   desktop_d8_fig5_data(5)     Kurztest
%
%   Die alte Fig. 5 (fig/optimized_ppo_40hz_trajectory.png im Paper-Repo) und
%   Figures/Simulation/optimized_ppo_circular_trajectoy_average.png (21.04.) zeigen sehr wahrscheinlich das
%   Basis-PPO (SpaceKinova_PPO_agent_motionprofile.mat), nicht Optimized.mat: calculate_kpi_spacekinova.m lud
%   damals fest diese Datei, und nur das Basis-PPO folgt der Bahn bis zum Ende (Befund A51).
%   Agenten: Optimized (Bayes, ohne CDR), CDR2-4 (Bayes und CDR, Hardware), PPO_base. 40 Hz, SK_desktop, 65 kg,
%   Halbkreis 8,5 s, stochastische Policy (wie die alten Auswertungen) und je eine deterministische Episode.
%   Ergebnis: data/simulation/desktop/D8_fig5_paths.mat (-v7, lesbar mit scipy) fuer fig/src/plot_fig5_paths.py

if nargin < 1, nEp = 50; end
setup_project;
desktop_build_model();

ag = struct('label', {'Optimized', 'CDR2-4', 'PPO_base'}, ...
    'file', {sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat'), ...
             sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'SpaceKinova_PPO_agent_motionprofile.mat')});
tg = (0:0.025:8.5)';
ref = [0 + 0.2 * sin(pi / 8.5 * tg), -0.025 + 0 * tg, 1.487 + 0.2 * cos(pi / 8.5 * tg)];
out = struct('t', tg, 'ref', ref, 'labels', {{ag.label}}, 'nEp', nEp);
for a = 1:numel(ag)
    P = nan(numel(tg), 3, nEp);
    k1 = nan(nEp, 1); mx = nan(nEp, 1); done = false(nEp, 1);
    for k = 1:nEp
        r = desktop_run_episode(desktop_config('agentFile', ag(a).file, 'agentLabel', ag(a).label, ...
            'explore', true, 'seed', k));
        e = interp1(r.ts.t_ee, r.ts.ep, tg, 'linear', NaN);
        P(:, :, k) = ref + e;
        k1(k) = r.metrics.mse_ee; mx(k) = r.metrics.max_ee; done(k) = ~r.metrics.early_stop;
    end
    rd = desktop_run_episode(desktop_config('agentFile', ag(a).file, 'agentLabel', ag(a).label));
    f = matlab.lang.makeValidName(ag(a).label);
    out.(f) = struct('mean', mean(P, 3, 'omitnan'), 'std', std(P, 0, 3, 'omitnan'), ...
        'det', ref + interp1(rd.ts.t_ee, rd.ts.ep, tg, 'linear', NaN), 'K1_mean', mean(k1), 'K1_std', std(k1), ...
        'K2_mean', mean(mx), 'completion', mean(done), 'K1_det', rd.metrics.mse_ee, 'K2_det', rd.metrics.max_ee);
    fprintf('%s: K1 %.5f +- %.5f m^2, K2 %.3f m, Abschluss %.2f | deterministisch K1 %.5f, K2 %.3f\n', ag(a).label, ...
        mean(k1), std(k1), mean(mx), mean(done), rd.metrics.mse_ee, rd.metrics.max_ee);
end
save(sk_path('data', 'simulation', 'desktop', 'D8_fig5_paths.mat'), '-struct', 'out', '-v7');
fprintf('Gespeichert: %s\n', sk_path('data', 'simulation', 'desktop', 'D8_fig5_paths.mat'));
end

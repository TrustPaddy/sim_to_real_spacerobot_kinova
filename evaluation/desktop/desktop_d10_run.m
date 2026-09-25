function desktop_d10_run(seeds, ep10, withRef)
%DESKTOP_D10_RUN  D10 komplett: Agentenpaare trainieren, dann wie D1 bei 40 und 10 Hz auswerten.
%   desktop_d10_run()                Seed 0, je 1000 Episoden, Optimized als Referenz (Lauf vom 25.09.)
%   desktop_d10_run([1 2], 4000, false)
%                                    Seeds 1 und 2, 10 Hz mit 4000 Episoden (gleiches Budget an
%                                    Agentenschritten wie 1000 Episoden bei 40 Hz), ohne Referenz
%
%   Die 40-Hz-Agenten trainieren immer 1000 Episoden. Auswertung mit desktop_d1_rates (Saetze nominal,
%   stoch, mass, delay, je 20 Ziehungen, Solver 5 ms). Optimized.mat laeuft auf Wunsch als Referenz mit.
%   Ergebnis: data/simulation/desktop/D10_eval_seed<Seeds>[_ep<ep10>]_<Zeit>.*

if nargin < 1, seeds = 0; end
if nargin < 2, ep10 = 1000; end
if nargin < 3, withRef = true; end

setup_project;
seedTag = sprintf('%d', seeds);
epTag = '';
if ep10 ~= 1000, epTag = sprintf('_ep%d', ep10); end
logFile = sk_path('data', 'simulation', 'desktop', sprintf('D10_log_seed%s%s_%s.txt', seedTag, epTag, ...
    char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'))));
diary(logFile);
cleanup = onCleanup(@() diary('off'));

tAll = tic;
ag = struct('label', {}, 'file', {}, 'trainHz', {});
for s = seeds
    r40 = desktop_d10_train(40, s, 1000);
    r10 = desktop_d10_train(10, s, ep10);
    ag(end + 1) = struct('label', sprintf('D10_40hz_s%d', s), 'file', r40.agentFile, 'trainHz', 40); %#ok<AGROW>
    ag(end + 1) = struct('label', sprintf('D10_10hz_s%d%s', s, epTag), 'file', r10.agentFile, 'trainHz', 10); %#ok<AGROW>
end
delete(gcp('nocreate'));

if withRef
    ag(end + 1) = struct('label', 'Optimized', ...
        'file', sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), 'trainHz', 40);
end
desktop_d1_rates(20, ag, sprintf('D10_eval_seed%s%s', seedTag, epTag));
fprintf('D10 komplett nach %.1f min\n', toc(tAll) / 60);
end

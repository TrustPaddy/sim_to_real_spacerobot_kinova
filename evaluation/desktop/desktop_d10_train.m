function out = desktop_d10_train(rateHz, seed, nEpisodes, tag)
%DESKTOP_D10_TRAIN  Trainiert einen PPO-Agenten fuer den Ratenvergleich (Desktop-Plan D10, R1.6, A21).
%   out = desktop_d10_train(40, 0)          voller Lauf, 1000 Episoden, 40 Hz, Seed 0
%   out = desktop_d10_train(10, 0)          dasselbe bei 10 Hz
%   out = desktop_d10_train(10, 0, 4000)    10 Hz mit demselben Budget an Agentenschritten wie 1000
%                                           Episoden bei 40 Hz (Datei mit Zusatz _ep4000)
%   out = desktop_d10_train(40, 0, 16, 'test')   Kurztest, speichert nur unter data/.../_test
%
%   Beide Raten nutzen dasselbe Modell (SK_desktop), dieselbe Bahn (Halbkreis 8,5 s, Referenz nach Zeit),
%   65 kg Basis, keine Verzoegerung, kein CDR und dieselben Agenten-Optionen wie Optimized.mat (bestes
%   Bayes-Trial, gerundet): ExperienceHorizon 600, MiniBatchSize 200, NumEpoch 10, ClipFactor 0,2,
%   DiscountFactor 0,99, GAE 0,95, Entropie 1e-3, Lernraten 5,7e-5 (Actor) und 1e-3 (Critic), 2 x 128 ReLU.
%   Die Optionen gelten pro Agentenschritt (Entscheidung 25.09.2026). Der zeitliche Horizont von
%   DiscountFactor und ExperienceHorizon ist bei 10 Hz deshalb viermal laenger.
%   Verschieden sind nur Agentenrate und Solver-Schritt: 40 Hz mit 5 ms, 10 Hz mit 20 ms wie im Training
%   von ppo_10hz. Filter und Rate Limiter rechnen im Agenten-Takt.
%   Paralleles Training (async, alle Worker des lokalen Pools). Asynchrones Training ist auch mit festem
%   Seed nicht bitgenau wiederholbar.
%
%   Ergebnis:
%     SavedAgents/MotionProfile/D10/D10_ppo_<rate>hz_seed<seed>.mat   agent, stats, info
%     data/simulation/desktop/D10_train_<rate>hz_seed<seed>_<Zeit>.csv  Lernkurve je Episode

if nargin < 2, seed = 0; end
if nargin < 3 || isempty(nEpisodes), nEpisodes = 1000; end
if nargin < 4, tag = ''; end
assert(ismember(rateHz, [10 40]), 'desktop_d10_train:rate', 'rateHz muss 10 oder 40 sein');

setup_project;
desktop_build_model();
mdl = 'SK_desktop';
if ~bdIsLoaded(mdl), load_system(mdl); end
set_param(mdl, 'SimMechanicsOpenEditorOnUpdate', 'off');

cfg = struct();
cfg.rateHz = rateHz;
cfg.seed = seed;
cfg.nEpisodes = nEpisodes;
cfg.Ts_agent = 1 / rateHz;
cfg.Ts = 0.005 * (rateHz == 40) + 0.02 * (rateHz == 10);
cfg.T = 8.5;
cfg.base_mass = 65;
cfg.hidden = 128;
cfg.opts = struct('ExperienceHorizon', 600, 'MiniBatchSize', 200, 'NumEpoch', 10, 'ClipFactor', 0.2, ...
    'DiscountFactor', 0.99, 'GAEFactor', 0.95, 'EntropyLossWeight', 1e-3, 'ActorLR', 5.7e-5, 'CriticLR', 1e-3);

% --- Workspace wie in desktop_run_episode (Standardwerte, keine Stoerung) ---
ec = desktop_config('Ts', cfg.Ts, 'Ts_agent', cfg.Ts_agent, 'T', cfg.T, 'base_mass', cfg.base_mass);
[EE_ref, EE_vref] = makeReference(ec);
vars = struct('EE_ref', EE_ref, 'EE_vref', EE_vref, 'reward_init', 0, 'isdone_init', 0, ...
    'p_Ts', ec.Ts, 'p_Ts_agent', ec.Ts_agent, 'p_T', ec.T, 'p_base_mass', ec.base_mass, ...
    'p_delay_steps', 0, 'p_damp_scale', 1, 'p_slew', ec.slew, 'p_cmd_scale', 1, 'p_obs_mode', 0, ...
    'p_obs_noise', zeros(29, 1));
fn = fieldnames(vars);
for k = 1:numel(fn)
    assignin('base', fn{k}, vars.(fn{k}));
end

% --- Spezifikationen wie Optimized.mat (gleiche Grenzen und Namen) ---
ref = load(sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat'), 'agent');
obsInfo = getObservationInfo(ref.agent);
actInfo = getActionInfo(ref.agent);

% --- Agent ---
rng(seed, 'twister');
initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hidden);
agent = rlPPOAgent(obsInfo, actInfo, initOpts);
o = agent.AgentOptions;
o.SampleTime = cfg.Ts_agent;
o.ExperienceHorizon = cfg.opts.ExperienceHorizon;
o.MiniBatchSize = cfg.opts.MiniBatchSize;
o.NumEpoch = cfg.opts.NumEpoch;
o.ClipFactor = cfg.opts.ClipFactor;
o.DiscountFactor = cfg.opts.DiscountFactor;
o.GAEFactor = cfg.opts.GAEFactor;
o.EntropyLossWeight = cfg.opts.EntropyLossWeight;
o.ActorOptimizerOptions.LearnRate = cfg.opts.ActorLR;
o.CriticOptimizerOptions.LearnRate = cfg.opts.CriticLR;
agent.AgentOptions = o;
assignin('base', 'agent', agent);

% --- Umgebung ---
env = rlSimulinkEnv(mdl, [mdl '/RL_Agent'], obsInfo, actInfo);
env.ResetFcn = @(in) setVariable(setVariable(in, 'reward_init', 0), 'isdone_init', 0);

% --- Training ---
par = rl.option.ParallelTraining('Mode', 'async');
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', nEpisodes, ...
    'MaxStepsPerEpisode', floor(cfg.T / cfg.Ts_agent), ...
    'ScoreAveragingWindowLength', 25, ...
    'StopTrainingCriteria', 'EpisodeCount', ...
    'StopTrainingValue', nEpisodes, ...
    'Plots', 'none', ...
    'Verbose', true, ...
    'StopOnError', 'off', ...
    'UseParallel', true, ...
    'ParallelizationOptions', par);

pool = gcp('nocreate');
if isempty(pool)
    try
        pool = parpool('Processes');
    catch err
        % Direkt nach einem vorigen Pool starten die Worker gelegentlich nicht (25.09.). Einmal neu versuchen.
        warning('desktop_d10_train:pool', 'Pool-Start fehlgeschlagen (%s), neuer Versuch in 30 s', err.message);
        pause(30);
        pool = parpool('Processes');
    end
end
fprintf('D10: %d Hz, Seed %d, %d Episoden, %d Worker, Solver %g s\n', rateHz, seed, nEpisodes, ...
    pool.NumWorkers, cfg.Ts);

tStart = tic;
stats = train(agent, env, trainOpts);
wallTime = toc(tStart);
fprintf('D10: Training fertig nach %.1f min\n', wallTime / 60);

% --- Speichern ---
info = struct('wallTime_s', wallTime, 'numWorkers', pool.NumWorkers, 'matlab', version, ...
    'git', gitInfo(), 'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    'computer', getenv('COMPUTERNAME'));
curve = table((1:numel(stats.EpisodeReward)).', stats.EpisodeReward(:), stats.AverageReward(:), ...
    stats.EpisodeQ0(:), stats.EpisodeSteps(:), 'VariableNames', ...
    {'episode', 'reward', 'avg_reward', 'q0', 'steps'});

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
name = sprintf('D10_ppo_%dhz_seed%d', rateHz, seed);
if nEpisodes ~= 1000
    % Abweichende Episodenzahl, z. B. 4000 bei 10 Hz fuer dasselbe Budget an Agentenschritten wie 1000 bei 40 Hz
    name = sprintf('%s_ep%d', name, nEpisodes);
end
dataDir = sk_path('data', 'simulation', 'desktop');
if isempty(tag)
    agentDir = sk_path('SavedAgents', 'MotionProfile', 'D10');
else
    agentDir = fullfile(dataDir, '_test');
    name = [name '_' tag];
end
if ~isfolder(agentDir), mkdir(agentDir); end
if ~isfolder(dataDir), mkdir(dataDir); end
agentFile = fullfile(agentDir, [name '.mat']);
save(agentFile, 'agent', 'cfg', 'info', 'curve');
csvDir = dataDir;
if ~isempty(tag), csvDir = agentDir; end
csvFile = fullfile(csvDir, sprintf('%s_train_%s.csv', strrep(name, 'D10_ppo', 'D10'), stamp));
writetable(curve, csvFile);
fprintf('Gespeichert: %s\n            %s\n', agentFile, csvFile);

out = struct('agentFile', agentFile, 'csvFile', csvFile, 'cfg', cfg, 'info', info, 'curve', curve);
end

% =====================================================================
function [EE_ref, EE_vref] = makeReference(cfg)
% Halbkreis nach Zeit wie desktop_run_episode (ref_timing 'wall')
t = 0:cfg.Ts:cfg.T;
omega = pi / cfg.T_path;
x = cfg.center(1) + cfg.r * sin(omega * t);
y = cfg.center(2) + 0 * t;
z = cfg.center(3) + cfg.r * cos(omega * t);
traj = [x(:), y(:), z(:)];
vref = [zeros(1, 3); diff(traj) / mean(diff(t))];
EE_ref = timeseries(traj, t);
EE_vref = timeseries(vref, t);
end

function g = gitInfo()
g = struct('hash', '', 'dirty', NaN);
[s1, h] = system(sprintf('git -C "%s" rev-parse --short HEAD', sk_path()));
[s2, d] = system(sprintf('git -C "%s" status --porcelain', sk_path()));
if s1 == 0, g.hash = strtrim(h); end
if s2 == 0, g.dirty = ~isempty(strtrim(d)); end
end

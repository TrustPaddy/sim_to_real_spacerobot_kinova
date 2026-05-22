%% SpaceKinovaDynamic_PPO_CDR.m
% PPO-Training fuer SpaceKinova mit Curriculum Domain Randomization.
%
% Design-Entscheidungen:
%   * Start q0  : Anker = cfg.q_start_anchor (per Hand definiert).
%                 Stoerung im Joint-Space: q_start_anchor + sigma_start*randn.
%   * Target EE : Anker = cfg.target_pos (per IK -> q_target).
%                 Stoerung im Joint-Space um q_target, FK liefert kartesische
%                 Zielposition. Damit ist Erreichbarkeit per Konstruktion
%                 garantiert (kein IK-Rejection noetig).
%
% VERSION: SEQUENTIELL (UseParallel = false).

clc; clear; close all;
rng(0,'twister');

%% =========================
%  CONFIG
%  =========================
cfg = struct();

% ---- Dateien/Modelle ----
cfg.urdfFile   = "SpaceKinova.urdf";
cfg.mdl        = "SpaceKinova_MotionProfile_point";
cfg.agentBlk   = cfg.mdl + "/RL_Agent";
cfg.eeBodyName = "kinova_end_effector_link";

% ---- Freiheitsgrade ----
cfg.nJ = 7;

% ---- Simulations- & Agenten-Zeit ----
cfg.T        = 25;
cfg.Ts       = 0.02;
cfg.Ts_agent = 0.1;

% ---- Zielpunkt im URDF-Frame (Nominal) ----
cfg.target_pos = [0.479; -0.005; 1.136];
cfg.target_R   = eye(3);

% ---- Start-Konfiguration (Anker fuer q0-Randomisierung) ----
% [0 15 180 -130 0 55 90] in Grad.
cfg.q_start_anchor = deg2rad([0; 15; 180; -130; 0; 55; 90]);

% ---- Kinova Gen3 7-DOF Limits ----
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
cfg.dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];

cfg.safetyFactor = 0.7;
cfg.dq_max       = cfg.safetyFactor * cfg.dqLim;
cfg.tau_max      = [32; 32; 32; 32; 13; 13; 13];

cfg.d_safe   = 0.02;
cfg.dt_agent = cfg.Ts_agent;

% ---- Observation Limits ----
cfg.ePLim   = 1.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

% ---- PPO/Training ----
cfg.maxEpisodes = 2000;
cfg.hiddenUnits = 128;

cfg.saveDir = "savedAgents_spacekinova_cdr";
cfg.saveTag = "ppo_spacekinova_cdr_point";

% ---- Logging ----
cfg.logEvery = 25;   % alle N Episoden eine CDR-Zeile im Command Window

%% =========================
%  CURRICULUM
%  =========================
% Pro Stage:
%   start_noise_deg       : Std-Abw der q0-Stoerung um q_start_anchor [Grad]
%   target_noise_deg      : Std-Abw der q_target-Stoerung [Grad].
%                           Resultierender EE-Offset folgt aus Jacobi.
%   full_rand_start_frac  : P(vollrandomisiertes q0 in weichen Limits).
%                           Gilt NUR fuer Start, nicht fuers Target.
%   min_episodes          : Mindestepisoden bis Aufstieg

cfg.cdr.stages = struct( ...
    'start_noise_deg',      {  2.0,   5.0,   10.0,  20.0, 40.0 }, ...
    'target_noise_deg',     {  0.0,   3.0,    6.0,  12.0, 24.0 }, ...
    'full_rand_start_frac', {  0.0,   0.0,    0.1,   0.3,  1.0 }, ...
    'min_episodes',         {   100,  300,    600,  1000,  inf } );

cdr_state = struct('stage', 1, 'episodes_in_stage', 0, 'episode_total', 0);
assignin('base','cdr_state', cdr_state);

cdr_log = struct( ...
    'episode',               [], ...
    'stage',                 [], ...
    'start_noise_deg',       [], ...
    'target_noise_deg',      [], ...
    'full_rand_start',       [], ...
    'q0_norm_from_anchor',   [], ...
    'target_ee_offset_norm', [] );
assignin('base','cdr_log', cdr_log);

%% =========================
% 1) Parameter in Base Workspace
% =========================
assignin('base','d_safe',     cfg.d_safe);
assignin('base','dq_max',     cfg.dq_max);
assignin('base','tau_max',    cfg.tau_max);
assignin('base','dt_agent',   cfg.dt_agent);
assignin('base','nJ',         cfg.nJ);
assignin('base','qLim_lower', cfg.qLim_lower);
assignin('base','qLim_upper', cfg.qLim_upper);
assignin('base','dqLim',      cfg.dqLim);

%% =========================
% 2) Konstante Referenz (Nominalziel)
% =========================
t = (0:cfg.Ts:cfg.T)';
N = numel(t);

EE_ref  = timeseries(repmat(cfg.target_pos.', N, 1), t);
EE_vref = timeseries(zeros(N, 3), t);
assignin('base','EE_ref',  EE_ref);
assignin('base','EE_vref', EE_vref);

%% =========================
% 3) Robot import + IK fuer Anker-Konfiguration q_target
% =========================
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);

robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
assignin('base','robot_rbt',  robot_rbt);
assignin('base','eeBodyName', cfg.eeBodyName);

ik = inverseKinematics("RigidBodyTree", robot_rbt);
ikWeights = [0.05 0.05 0.05  1 1 1];
qSeed = homeConfiguration(robot_rbt);
Tgoal = [cfg.target_R cfg.target_pos; 0 0 0 1];
[qSol, ~] = ik(cfg.eeBodyName, Tgoal, ikWeights, qSeed);
q_target = qSol(1:cfg.nJ).';
assignin('base','q_target', q_target);

% Anker-Objekte in cfg legen damit die ResetFcn ohne evalin auskommt
cfg.q_target  = q_target;
cfg.robot_rbt = robot_rbt;

% Sanity-Check der Anker
T_start  = getTransform(robot_rbt, cfg.q_start_anchor.', cfg.eeBodyName);
T_target = getTransform(robot_rbt, q_target.',           cfg.eeBodyName);
ee_start  = T_start(1:3,4);
ee_target = T_target(1:3,4);
fprintf("EE-Position bei q_start_anchor : [%.3f %.3f %.3f]\n", ee_start);
fprintf("EE-Position bei q_target (Soll): [%.3f %.3f %.3f]\n", ee_target);
fprintf("Distanz Start -> Ziel          : %.3f m\n", norm(ee_start - ee_target));

figure('Name','Anker-Konfigurationen');
subplot(1,2,1);
show(robot_rbt, cfg.q_start_anchor.', 'PreservePlot', true); hold on;
plot3(ee_start(1), ee_start(2), ee_start(3), 'bo', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'b');
title('q\_start\_anchor');
subplot(1,2,2);
show(robot_rbt, q_target.', 'PreservePlot', true); hold on;
plot3(cfg.target_pos(1), cfg.target_pos(2), cfg.target_pos(3), 'go', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'g');
title('q\_target (IK-Loesung fuer cfg.target\_pos)');
drawnow;

% Default-IC fuer Compile-Time. setVariable in ResetFcn ueberschreibt das
% zur Laufzeit pro Episode.
assignin('base','q0', cfg.q_start_anchor);

%% =========================
% 4) Simulink Modell konfigurieren
% =========================
load_system(cfg.mdl);
set_param(cfg.mdl, ...
    'StopTime',   num2str(cfg.T), ...
    'Solver',     'ode14x', ...
    'FixedStep',  num2str(cfg.Ts), ...
    'SolverType', 'Fixed-step');
open_system(cfg.mdl);

%% =========================
% 5) Observation & Action Definition
% =========================
nJ = cfg.nJ;

obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
     cfg.qLim_lower; ...
    -cfg.dqLim; ...
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ];

obsHigh = [ ...
     cfg.ePLim   * ones(3,1); ...
     cfg.eVLim   * ones(3,1); ...
     cfg.qLim_upper; ...
     cfg.dqLim; ...
     cfg.vBLim   * ones(3,1); ...
     cfg.wBLim   * ones(3,1); ...
     cfg.eOriLim * ones(3,1) ];

obsInfo = rlNumericSpec([numel(obsLow) 1], ...
    'LowerLimit', obsLow, 'UpperLimit', obsHigh, 'Name', "obs");

actInfo = rlNumericSpec([nJ 1], ...
    'Name', "dq_cmd", ...
    'LowerLimit', -ones(nJ,1), ...
    'UpperLimit',  ones(nJ,1));

%% =========================
% 6) RL-Umgebung + ResetFcn
% =========================
env = rlSimulinkEnv(cfg.mdl, cfg.agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) localResetFunctionCDR(in, cfg);

%% =========================
% 7) PPO Agent
% =========================
initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
agent = rlPPOAgent(obsInfo, actInfo, initOpts);

agent.AgentOptions.SampleTime                       = cfg.Ts_agent;
agent.AgentOptions.ExperienceHorizon                = 1024;
agent.AgentOptions.MiniBatchSize                    = 512;
agent.AgentOptions.NumEpoch                         = 10;
agent.AgentOptions.ClipFactor                       = 0.2;
agent.AgentOptions.EntropyLossWeight                = 1e-3;
agent.AgentOptions.ActorOptimizerOptions.LearnRate  = 5e-04;
agent.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-03;
assignin('base','agent', agent);

%% =========================
% 8) Training (SEQUENTIELL)
% =========================
set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'off');

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',                cfg.maxEpisodes, ...
    'MaxStepsPerEpisode',         floor(cfg.T/cfg.Ts_agent), ...
    'ScoreAveragingWindowLength', 25, ...
    'StopTrainingCriteria',       "AverageReward", ...
    'StopTrainingValue',          900, ...
    'Plots',                      "training-progress", ...
    'StopOnError',                "off", ...
    'UseParallel',                false );

trainingStats = train(agent, env, trainOpts);

set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'on');

%% =========================
% 9) Save Agent + Stats + CDR-Log
% =========================
if ~exist(cfg.saveDir,'dir'); mkdir(cfg.saveDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS'); %#ok<DATST>
saveFile = fullfile(cfg.saveDir, sprintf("%s_%s.mat", cfg.saveTag, stamp));
cdr_log_final = evalin('base','cdr_log');
cfg_save = rmfield(cfg, 'robot_rbt');  % robot_rbt nicht mit speichern
save(saveFile, 'agent', 'trainingStats', 'cdr_log_final', 'cfg_save');
fprintf("\nGespeichert: %s\n", saveFile);

%% =========================
% 10) Abspielen + Stats
% =========================
simOpts = rlSimulationOptions('MaxSteps', floor(cfg.T/cfg.Ts_agent));
simOut  = sim(env, agent, simOpts); %#ok<NASGU>

tail_idx = round(0.7*cfg.maxEpisodes):cfg.maxEpisodes;
tail_idx = tail_idx(tail_idx <= numel(trainingStats.EpisodeReward));
if ~isempty(tail_idx)
    CV    = std(trainingStats.EpisodeReward(tail_idx)) ...
          / max(eps, abs(mean(trainingStats.EpisodeReward(tail_idx))));
    drift = mean(diff(trainingStats.AverageReward(tail_idx)));
    fprintf("\nKonsistenz (CV tail)   : %.4f\n", CV);
    fprintf("Konvergenz (drift tail): %.4f\n", drift);
end
[bestAvg, iBest] = max(trainingStats.AverageReward);
fprintf("Bester AvgReward       : %.2f @ Episode %d\n", bestAvg, iBest);

%% =========================
% 11) Curriculum-Diagnose-Plot
% =========================
cdr_log_final = evalin('base','cdr_log');
if ~isempty(cdr_log_final.episode)
    figure('Name','CDR Diagnose');
    subplot(3,1,1);
    plot(cdr_log_final.episode, cdr_log_final.stage, 'k.-'); grid on;
    ylabel('Stage'); title('Curriculum-Stage je Episode');
    subplot(3,1,2);
    plot(cdr_log_final.episode, cdr_log_final.target_ee_offset_norm, 'b.');
    grid on;
    ylabel('|EE-Ziel-Offset| [m]'); title('Ziel-Randomisierung (via FK)');
    subplot(3,1,3);
    plot(cdr_log_final.episode, cdr_log_final.q0_norm_from_anchor, 'r.');
    grid on;
    ylabel('||q0 - q_{start anchor}|| [rad]'); xlabel('Episode');
    title('Start-Randomisierung');
end


%% =========================================================================
%  LOKALE RESET-FUNKTION
%  =========================================================================
function in = localResetFunctionCDR(in, cfg)

% ---- Curriculum-State holen ----
try
    cdr_state = evalin('base','cdr_state');
catch
    cdr_state = struct('stage', 1, 'episodes_in_stage', 0, 'episode_total', 0);
end

cdr_state.episodes_in_stage = cdr_state.episodes_in_stage + 1;
cdr_state.episode_total     = cdr_state.episode_total + 1;
stage    = cdr_state.stage;
stageDef = cfg.cdr.stages(stage);

% Stage-Aufstieg rein episodenbasiert
if cdr_state.episodes_in_stage >= stageDef.min_episodes ...
        && stage < numel(cfg.cdr.stages)
    cdr_state.stage             = stage + 1;
    cdr_state.episodes_in_stage = 1;     % diese Episode zaehlt in neue Stage
    stage    = cdr_state.stage;
    stageDef = cfg.cdr.stages(stage);
    fprintf("[CDR] Aufstieg in Stage %d (Episode %d)\n", ...
            stage, cdr_state.episode_total);
end

nJ = cfg.nJ;

% ---------------------------------------------------------------
% q0 randomisieren  (um cfg.q_start_anchor)
% ---------------------------------------------------------------
full_rand_start = (rand() < stageDef.full_rand_start_frac);
if full_rand_start
    margin = 0.05 * (cfg.qLim_upper - cfg.qLim_lower);
    lo = cfg.qLim_lower + margin;
    hi = cfg.qLim_upper - margin;
    q0 = lo + (hi - lo) .* rand(nJ,1);
else
    sigma_start = deg2rad(stageDef.start_noise_deg);
    q0 = cfg.q_start_anchor + sigma_start * randn(nJ,1);
    q0 = max(min(q0, cfg.qLim_upper), cfg.qLim_lower);
end

% ---------------------------------------------------------------
% Zielpunkt via Joint-Space-Stoerung + FK
% ---------------------------------------------------------------
% Sample q_target_ep = q_target + sigma * randn, dann FK liefert
% target_pos_ep. Erreichbarkeit per Konstruktion gegeben.
sigma_tgt = deg2rad(stageDef.target_noise_deg);
if sigma_tgt > 0
    q_target_ep = cfg.q_target + sigma_tgt * randn(nJ,1);
    q_target_ep = max(min(q_target_ep, cfg.qLim_upper), cfg.qLim_lower);
else
    q_target_ep = cfg.q_target;
end
T_ee = getTransform(cfg.robot_rbt, q_target_ep.', cfg.eeBodyName);
target_pos_ep = T_ee(1:3,4);

% Referenz-Timeseries
N = numel(0:cfg.Ts:cfg.T);
t_ep = (0:cfg.Ts:cfg.T)';
EE_ref_ep  = timeseries(repmat(target_pos_ep.', N, 1), t_ep);
EE_vref_ep = timeseries(zeros(N, 3),                  t_ep);

% In SimulationInput schreiben
in = setVariable(in,'q0',      q0);
in = setVariable(in,'EE_ref',  EE_ref_ep);
in = setVariable(in,'EE_vref', EE_vref_ep);

% ---------------------------------------------------------------
% Logging
% ---------------------------------------------------------------
try
    cdr_log = evalin('base','cdr_log');
catch
    cdr_log = struct( ...
        'episode',[], 'stage',[], 'start_noise_deg',[], 'target_noise_deg',[], ...
        'full_rand_start',[], 'q0_norm_from_anchor',[], ...
        'target_ee_offset_norm',[]);
end
ep_offset_norm = norm(target_pos_ep - cfg.target_pos);
q0_anchor_norm = norm(q0 - cfg.q_start_anchor);
cdr_log.episode(end+1,1)               = cdr_state.episode_total;
cdr_log.stage(end+1,1)                 = stage;
cdr_log.start_noise_deg(end+1,1)       = stageDef.start_noise_deg;
cdr_log.target_noise_deg(end+1,1)      = stageDef.target_noise_deg;
cdr_log.full_rand_start(end+1,1)       = full_rand_start;
cdr_log.q0_norm_from_anchor(end+1,1)   = q0_anchor_norm;
cdr_log.target_ee_offset_norm(end+1,1) = ep_offset_norm;
assignin('base','cdr_log', cdr_log);

% Periodisches Print
if mod(cdr_state.episode_total, cfg.logEvery) == 0
    fprintf(['[CDR] ep=%4d  stage=%d  start_sigma=%4.1f deg  ' ...
             'full_rand=%d  tgt_sigma=%4.1f deg  ' ...
             '|tgt_off|=%5.3f m  ||dq0||=%5.3f rad\n'], ...
            cdr_state.episode_total, stage, stageDef.start_noise_deg, ...
            full_rand_start, stageDef.target_noise_deg, ...
            ep_offset_norm, q0_anchor_norm);
end

assignin('base','cdr_state', cdr_state);
end
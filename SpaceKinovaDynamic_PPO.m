%% SpaceKinovaDynamic_PPO.m
% Trainiert einen PPO-Agenten für einen frei schwebenden "SpaceKinova"
% (Würfelbasis + Kinova-Arm) basierend auf deinem SpaceRobotDynamic.m.
%
% WICHTIG:
% - Dieses Skript baut die RL-Umgebung (obs/act specs), Referenztrajektorie,
%   IK-Preprocessing (q_des), und startet das Training.
% - Du musst dein Simulink-Modell entsprechend angepasst haben:
%     * Robot-Subsystem nutzt SpaceKinova.urdf (oder direkt Simscape-Import)
%     * RL_Agent Block existiert unter mdl/RL_Agent
%     * Reward/Done und Collision-Monitor sind auf nJ=7 und dq_cmd ausgelegt
%
% Minimal anzupassen (oben im CONFIG-Block):
%   - cfg.mdl, cfg.agentBlk
%   - cfg.urdfFile
%   - cfg.eeBodyName (optional)
%
% Autor: ChatGPT (Template zum direkten Übernehmen)

clc; clear; close all;
rng(0,'twister');

%% =========================
%  CONFIG
%  =========================
cfg = struct();

% ---- Dateien/Modelle ----
cfg.urdfFile   = "SpaceKinova.urdf";   % <- deine generierte URDF
cfg.mdl        = "SpaceKinova";        % <- dein angepasstes Simulink-Modell (z.B. Kopie von SpaceRobot.slx)
cfg.agentBlk   = cfg.mdl + "/RL_Agent";

% Falls leer: nimmt den letzten Body als EE
cfg.eeBodyName = "";                  % z.B. "ee_link" oder "" für auto

% ---- Freiheitsgrade ----
cfg.nJ = 7;

% ---- Simulations- & Agenten-Zeit ----
cfg.T        = 8.5;     % Episodendauer [s]
cfg.Ts       = 0.01;    % Simulations-FixedStep [s]
cfg.Ts_agent = 0.1;     % Agent SampleTime [s] (muss zu RL-Block passen!)

% ---- Referenztrajektorie (Kreis) ----
cfg.r      = 0.4;                         % Radius [m]
cfg.center = [4.5 - cfg.r, 0.0, 0.0];     % Mittelpunkt
cfg.omega  = pi/cfg.T;                    % Winkelgeschwindigkeit
cfg.zConst = 0.0;                         % konstante z-Höhe

% ---- Sicherheits-/Spec-Parameter ----
cfg.d_safe    = 0.02;   % Mindestabstand [m] (Collision Monitor)
cfg.dq_max    = 0.8;    % |dq_cmd| max (für Actionspec + Simulink Sättigung)
cfg.tau_max   = 30;     % optional: falls du im Simulink einen inneren Velocity-Regler hast (Torque Saturation)
cfg.dt_agent  = 0.05;   % nur falls du dt_agent im Modell benutzt (sonst ignorieren)

% Joint Limits (als Beispiel; bitte an Kinova anpassen!)
% - Für Kinova Gen3 sind echte Limits typischerweise größer/anders; setz das passend zu deinem URDF/Robot.
cfg.qLim_abs  = pi;     % rad (Fallback-Limit, falls du keine individuellen Grenzen nutzt)
cfg.dqLim_abs = 2.0;    % rad/s (für Observation clipping)

% Observation Limits für Basis/Fehler wie in deinem ursprünglichen Script
cfg.ePLim   = 0.5;      % [m]
cfg.eVLim   = 1.0;      % [m/s]
cfg.vBLim   = 0.5;      % [m/s]
cfg.wBLim   = 1.0;      % [rad/s]
cfg.eOriLim = pi;       % [rad]

% PPO/Training
cfg.maxEpisodes = 1000;
cfg.hiddenUnits = 128;

% Speicherpfade
cfg.saveDir = "savedAgents_spacekinova";
cfg.saveTag = "ppo_spacekinova_vel";

%% =========================
% 1) Parameter in Base Workspace (für Simulink-Blöcke)
% =========================
assignin('base','d_safe',   cfg.d_safe);
assignin('base','dq_max',   cfg.dq_max);
assignin('base','tau_max',  cfg.tau_max);
assignin('base','dt_agent', cfg.dt_agent);
assignin('base','nJ',       cfg.nJ);

% Optional: wenn dein Modell mit q_lim arbeitet
assignin('base','qLim_abs',  cfg.qLim_abs);
assignin('base','dqLim_abs', cfg.dqLim_abs);

%% =========================
% 2) Referenztrajektorie erzeugen (EE_ref, EE_vref)
% =========================
t = 0:cfg.Ts:cfg.T;

x = cfg.center(1) + cfg.r*cos(cfg.omega*t);
y = cfg.center(2) + cfg.r*sin(cfg.omega*t);
z = cfg.center(3) + cfg.zConst*t;

traj = [x(:) y(:) z(:)];

dt  = mean(diff(t));
vref = [zeros(1,3); diff(traj)/dt];

EE_ref  = timeseries(traj, t);
EE_vref = timeseries(vref, t);

assignin('base','EE_ref',  EE_ref);
assignin('base','EE_vref', EE_vref);

%% =========================
% 3) Robot import + IK Preprocessing (q_des)
% =========================
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);

robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

if strlength(cfg.eeBodyName) == 0
    cfg.eeBodyName = string(robot_rbt.BodyNames{end});
end

assignin('base','robot_rbt', robot_rbt);
assignin('base','eeBodyName', cfg.eeBodyName);

% IK vorbereiten (nur als Startseed/Reset-Hilfe; Training nutzt weiterhin Dynamik aus Simulink)
ik = inverseKinematics("RigidBodyTree", robot_rbt);

% Position ist wichtig, Orientierung nur schwach (damit IK robust bleibt)
ikWeights = [1 1 1 0.05 0.05 0.05];

qSeed = homeConfiguration(robot_rbt);
q_des = zeros(numel(t), cfg.nJ);

% Zielorientierung: Identität
R0 = eye(3);

for k = 1:numel(t)
    Tgoal = [R0 traj(k,:).'; 0 0 0 1];
    [qSol, solInfo] = ik(cfg.eeBodyName, Tgoal, ikWeights, qSeed); %#ok<ASGLU>
    q_des(k,:) = qSol(1:cfg.nJ);
    qSeed = qSol;
end

assignin('base','q_des', q_des);

%% =========================
% 4) Simulink Modell konfigurieren
% =========================
load_system(cfg.mdl);

set_param(cfg.mdl, ...
    'StopTime',   num2str(cfg.T), ...
    'Solver',     'ode4', ...
    'FixedStep',  num2str(cfg.Ts), ...
    'SolverType', 'Fixed-step');

open_system(cfg.mdl);

%% =========================
% 5) Observation & Action Definition
% =========================
nJ = cfg.nJ;

% Observation: [ep(3); ev(3); q(nJ); dq(nJ); vbase(3); wbase(3); e_ori(3)]
obsDim = 3 + 3 + 2*nJ + 6 + 3;

obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
    -cfg.qLim_abs  * ones(nJ,1); ...
    -cfg.dqLim_abs * ones(nJ,1); ...
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ...
    ];

obsHigh = -obsLow;

obsInfo = rlNumericSpec([numel(obsLow) 1], ...
    'LowerLimit', obsLow, ...
    'UpperLimit', obsHigh, ...
    'Name', "obs");

% Action: dq_cmd (Joint velocity commands)
actInfo = rlNumericSpec([nJ 1], ...
    'Name', "dq_cmd", ...
    'LowerLimit', -cfg.dq_max*ones(nJ,1), ...
    'UpperLimit',  cfg.dq_max*ones(nJ,1));

%% =========================
% 6) RL-Umgebung verknüpfen + ResetFcn
% =========================
env = rlSimulinkEnv(cfg.mdl, cfg.agentBlk, obsInfo, actInfo);

% ResetFcn als lokale Funktion (siehe unten)
env.ResetFcn = @(in)localResetFunctionSpaceKinova(in, cfg.nJ);

% Optional: Episodes reproducible (wenn dein Reset randomisiert wird)
% env.ResetFcn = @(in)localResetFunctionSpaceKinova(in, cfg.nJ, "Randomize", true);

%% =========================
% 7) PPO Agent erstellen
% =========================
initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);

agent = rlPPOAgent(obsInfo, actInfo, initOpts);
agent.AgentOptions.SampleTime = cfg.Ts_agent;

% Optional: feinere PPO-Hyperparameter (wie in deinem Bericht kommentiert)
% agentOpts = rlPPOAgentOptions( ...
%     'SampleTime', cfg.Ts_agent, ...
%     'ExperienceHorizon', 1024, ...
%     'MiniBatchSize', 256, ...
%     'NumEpoch', 10, ...
%     'ClipFactor', 0.2, ...
%     'EntropyLossWeight', 1e-3, ...
%     'AdvantageEstimateMethod', 'gae', ...
%     'GAEFactor', 0.95, ...
%     'DiscountFactor', 0.995);
% agent = rlPPOAgent(actor, critic, agentOpts);

assignin('base','agent', agent);

%% =========================
% 8) Sanity Check
% =========================
try
    disp(getActionInfo(agent));
    disp(getObservationInfo(agent));
catch ME
    warning("%s: %s", ME.identifier, ME.message);
end

%% =========================
% 9) Training Options + Train
% =========================
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes', cfg.maxEpisodes, ...
    'MaxStepsPerEpisode', floor(cfg.T/cfg.Ts_agent), ...
    'ScoreAveragingWindowLength', 25, ...
    'StopTrainingCriteria', "AverageReward", ...
    'StopTrainingValue', -0, ...
    'Plots', "training-progress" ...
);

% Optional: Save best agent
% ts = datestr(now,'yyyymmdd_HHMMSS');
% saveDirRun = fullfile(cfg.saveDir, cfg.saveTag + "_" + string(ts));
% trainOpts.SaveAgentCriteria  = "AverageReward";
% trainOpts.SaveAgentValue     = -inf;
% trainOpts.SaveAgentDirectory = saveDirRun;

trainingStats = train(agent, env, trainOpts);

%% =========================
% 10) Agent speichern
% =========================
if ~isfolder(cfg.saveDir), mkdir(cfg.saveDir); end
timestamp = datestr(now,'yyyymmdd_HHMMSS');
outName = fullfile(cfg.saveDir, cfg.saveTag + "_" + string(timestamp) + ".mat");
save(outName, 'agent', 'cfg', 'trainingStats');

fprintf("\nGespeichert: %s\n", outName);

%% =========================
%  LOKALE RESET-FUNKTION
% =========================
function in = localResetFunctionSpaceKinova(in, nJ, varargin)
% Reset-Funktion für RL-Training:
% - setzt Startpose q0 (aus q_des falls vorhanden)
% - setzt dq0, base_v0, base_w0, phi0
%
% OPTIONAL: Randomize=true -> kleine Störungen auf q0/dq0

p = inputParser;
p.addParameter("Randomize", false, @(v)islogical(v)||ismember(v,[0 1]));
p.parse(varargin{:});
doRand = logical(p.Results.Randomize);

% --- Startpose (IK-Seed, falls vorhanden) ---
try
    q_start = evalin('base','q_des(1,:).'';'); % Spalte
    if numel(q_start) ~= nJ, error("BadSize"); end
catch
    q_start = zeros(nJ,1);
end

q0 = q_start;
dq0 = zeros(nJ,1);

if doRand
    q0  = q0  + deg2rad(1.0)*randn(nJ,1);
    dq0 = dq0 + 0.05*randn(nJ,1);
end

base_v0 = zeros(3,1);
base_w0 = zeros(3,1);
phi0    = 0;

% --- In Simulink schreiben ---
in = setVariable(in,'q0', q0);
in = setVariable(in,'dq0', dq0);
in = setVariable(in,'base_v0', base_v0);
in = setVariable(in,'base_w0', base_w0);
in = setVariable(in,'phi0', phi0);

% falls du Memory-Blöcke hast:
in = setVariable(in,'reward_init', 0);
in = setVariable(in,'isdone_init', 0);
end

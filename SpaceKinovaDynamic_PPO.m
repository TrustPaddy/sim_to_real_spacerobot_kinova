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
% Konfiguriert fuer Kinova Gen3 7-DOF (Spezifikationen aus ros_kortex URDF)

clc; clear; close all;
rng(0,'twister');

%% =========================
%  CONFIG
%  =========================
cfg = struct();

% ---- Dateien/Modelle ----
cfg.urdfFile   = "SpaceKinova.urdf";   % <- deine generierte URDF
%cfg.mdl        = "SpaceKinova";        % <- dein angepasstes Simulink-Modell (z.B. Kopie von SpaceRobot.slx)
%cfg.mdl        = "SpaceKinova_MotionProfile";        % <- dein angepasstes Simulink-Modell (z.B. Kopie von SpaceRobot.slx)
cfg.mdl        = "SpaceKinova_MotionProfile";        % <- dein angepasstes Simulink-Modell (z.B. Kopie von SpaceRobot.slx)
cfg.agentBlk   = cfg.mdl + "/RL_Agent";

% Kinova Gen3 End-Effector Link (mit Prefix aus make_spacekinova_urdf)
cfg.eeBodyName = "kinova_end_effector_link";

% ---- Freiheitsgrade ----
cfg.nJ = 7;

% ---- Simulations- & Agenten-Zeit ----
%cfg.T        = 8.5;     % Episodendauer [s]
cfg.T        = 16;     % Episodendauer [s]
%cfg.Ts       = 0.005;    % Simulations-FixedStep [s]
cfg.Ts       = 0.02;    % Simulations-FixedStep [s]
%cfg.Ts_agent = 0.025;   % Agent SampleTime [s] = 40 Hz (Kinova Gen3 High-Level Servo Rate)
cfg.Ts_agent = 0.1;   % Agent SampleTime [s] = 40 Hz (Kinova Gen3 High-Level Servo Rate)

% ---- Referenztrajektorie (Kreis) ----
cfg.r      = 0.2;                         % Radius [m]
cfg.center = [0.479, -0.005, 0.936 + cfg.r];     % Mittelpunkt
cfg.omega  = pi/cfg.T;                    % Winkelgeschwindigkeit
cfg.yConst = 0;                         % konstante z-Höhe

% ---- Kinova Gen3 7-DOF Gelenkspezifikationen (aus ros_kortex URDF) ----
% Positionslimits: J1,J3,J7 continuous -> Software-Limit 2*pi
%                  J2: +/-2.41 rad, J4: +/-2.66 rad, J5: +/-2.23 rad, J6: +/-2.01 rad
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];  % [rad]
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];  % [rad]

% Geschwindigkeitslimits (aus URDF)
% Grosse Aktuatoren J1-J4: 1.3963 rad/s, Kleine Aktuatoren J5-J7: 1.2218 rad/s
cfg.dqLim = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218]; % [rad/s]

% Action-Limits: 70% der echten Limits (Sicherheitsfaktor)
cfg.safetyFactor = 0.7;
cfg.dq_max = cfg.safetyFactor * cfg.dqLim;
% -> [0.9774; 0.9774; 0.9774; 0.9774; 0.8553; 0.8553; 0.8553] rad/s

% Drehmomentlimits (Nominal/Continuous)
% Grosse Aktuatoren J1-J4: 32 Nm, Kleine Aktuatoren J5-J7: 13 Nm
cfg.tau_max = [32; 32; 32; 32; 13; 13; 13]; % [Nm]

% Sicherheits-Parameter
cfg.d_safe   = 0.02;          % Mindestabstand [m] (Collision Monitor)
cfg.dt_agent = cfg.Ts_agent;  % synchron mit Agent SampleTime

% Observation Limits für Basis/Fehler wie in deinem ursprünglichen Script
cfg.ePLim   = 0.5;      % [m]
cfg.eVLim   = 1.0;      % [m/s]
cfg.vBLim   = 0.5;      % [m/s]
cfg.wBLim   = 1.0;      % [rad/s]
cfg.eOriLim = pi;       % [rad]

% PPO/Training
cfg.maxEpisodes = 2000;
cfg.hiddenUnits = 128;

% Speicherpfade
cfg.saveDir = "savedAgents_spacekinova";
cfg.saveTag = "ppo_spacekinova_vel";

%% =========================
% 1) Parameter in Base Workspace (für Simulink-Blöcke)
% =========================
assignin('base','d_safe',     cfg.d_safe);
assignin('base','dq_max',     cfg.dq_max);      % 7x1 Vektor
assignin('base','tau_max',    cfg.tau_max);      % 7x1 Vektor
assignin('base','dt_agent',   cfg.dt_agent);
assignin('base','nJ',         cfg.nJ);
assignin('base','qLim_lower', cfg.qLim_lower);  % 7x1 Vektor
assignin('base','qLim_upper', cfg.qLim_upper);  % 7x1 Vektor
assignin('base','dqLim',      cfg.dqLim);        % 7x1 Vektor


% % PD velocity controller gains (innerer Regler in Simulink)
% cfg.Kp_vel = 50;    % Proportional-Verstaerkung
% cfg.Kd_vel = 1.0;   % Daempfung
% assignin('base', 'Kp_vel', cfg.Kp_vel);
% assignin('base', 'Kd_vel', cfg.Kd_vel);

%% =========================
% 2) Referenztrajektorie erzeugen (EE_ref, EE_vref)
% =========================
t = 0:cfg.Ts:cfg.T;

x = cfg.center(1) + cfg.r*sin(cfg.omega*t);
y = cfg.center(2) + cfg.yConst*t;
z = cfg.center(3) - cfg.r*cos(cfg.omega*t);

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

%Animation: jeden 10. Frame zeigen
for k = 1:10:size(q_des,1)/2
    show(robot_rbt, q_des(k,:), 'PreservePlot', false);
    hold on;
    plot3(traj(:,1), traj(:,2), traj(:,3), 'r--', 'LineWidth', 2);
    plot3(traj(k,1), traj(k,2), traj(k,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    title(sprintf('t = %.2f s', t(k)));
    drawnow;
    pause(0.005);
end

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

% Observation: [ep(3); ev(3); q(nJ); dq(nJ); vbase(3); wbase(3); e_ori(3)]
obsDim = 3 + 3 + 2*nJ + 6 + 3;

obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
    cfg.qLim_lower; ...              % per-Joint untere Positionslimits
    -cfg.dqLim; ...                  % per-Joint Geschwindigkeitslimits (negativ)
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ...
    ];

obsHigh = [ ...
    cfg.ePLim   * ones(3,1); ...
    cfg.eVLim   * ones(3,1); ...
    cfg.qLim_upper; ...              % per-Joint obere Positionslimits
    cfg.dqLim; ...                   % per-Joint Geschwindigkeitslimits (positiv)
    cfg.vBLim   * ones(3,1); ...
    cfg.wBLim   * ones(3,1); ...
    cfg.eOriLim * ones(3,1) ...
    ];

obsInfo = rlNumericSpec([numel(obsLow) 1], ...
    'LowerLimit', obsLow, ...
    'UpperLimit', obsHigh, ...
    'Name', "obs");

% Action: dq_cmd (Joint velocity commands, per-Joint begrenzt)
actInfo = rlNumericSpec([nJ 1], ...
    'Name', "dq_cmd", ...
    'LowerLimit', -ones(nJ,1), ...   % 7x1 Vektor
    'UpperLimit',  ones(nJ,1));      % 7x1 Vektor

%% =========================
% 6) RL-Umgebung verknüpfen + ResetFcn
% =========================
env = rlSimulinkEnv(cfg.mdl, cfg.agentBlk, obsInfo, actInfo);

% ResetFcn mit Randomisierung (wichtig fuer Sim-to-Real Robustheit)
env.ResetFcn = @(in)localResetFunctionSpaceKinova(in, cfg);

% % ============================================
% 7.1) PPO AGENT (Baseline)
% ============================================
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


% % ============================================
% 7.1) PPO AGENT (default-nah)
% ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlPPOAgent(obsInfo, actInfo, initOpts);
% 
% Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);
 
% %% ============================================
% % 7.2) TD3 AGENT (default-nah)
% % ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlTD3Agent(obsInfo, actInfo, initOpts);
% 
% % Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);

% %% ============================================
% % 7.3) SAC AGENT (default-nah)
% % ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlSACAgent(obsInfo, actInfo, initOpts);
% 
% 
% % Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);

% %% ============================================
% % 7.4) PG AGENT (default-nah)
% % ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlPGAgent(obsInfo, actInfo, initOpts);
% 
% % Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);
% 
% % ============================================
% 7.5) DDPG AGENT (default-nah)
% ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlDDPGAgent(obsInfo, actInfo, initOpts);
% 
% Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);

% %% ============================================
% % 7.6) TRPO AGENT (default-nah)
% % ============================================
% initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
% agent = rlTRPOAgent(obsInfo, actInfo, initOpts);
% 
% % Nur modellabhängige Einstellung setzen
% agent.AgentOptions.SampleTime = cfg.Ts_agent;
% 
% assignin('base','agent', agent);

%% =========================
% 8) Sanity Check
% =========================
try
    disp(getActionInfo(agent));
    disp(getObservationInfo(agent));
catch ME
    warning("%s: %s", ME.identifier, ME.message);
end

if isempty(gcp('nocreate'))
    parpool('local', 8);
end

%% =========================
% 9) Training Options + Train
% =========================

% Visualisierung während Training unterdrücken
set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'off');

trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',                cfg.maxEpisodes, ...
    'MaxStepsPerEpisode',         floor(cfg.T/cfg.Ts_agent), ...
    'ScoreAveragingWindowLength', 25, ...  
    'StopTrainingCriteria',       "AverageReward", ...  
    'StopTrainingValue',          1850, ...
    'Plots',                      "training-progress", ...
    'StopOnError',                "off",  ...
    'UseParallel', true, ...
    'ParallelizationOptions', rl.option.ParallelTraining(...
        'Mode', 'async') ...
);



% % Besten Agenten speichern (empfohlen!)
% saveDirRun = fullfile(cfg.saveDir, cfg.saveTag + "_" + string(datestr(now,'yyyymmdd_HHMMSS')));
% trainOpts.SaveAgentCriteria  = "EpisodeReward";
% trainOpts.SaveAgentValue     = -inf;
% trainOpts.SaveAgentDirectory = saveDirRun;

trainingStats = train(agent, env, trainOpts);

% NACH dem Training: Sauber abspielen
set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'on');  % Explorer wieder an

simOpts = rlSimulationOptions('MaxSteps', floor(cfg.T/cfg.Ts_agent));
simOut  = sim(env, agent, simOpts);  % Frische Simulation mit funktionierendem Explorer

%% =========================
% % 10) Agent speichern
% % =========================
% if ~isfolder(cfg.saveDir), mkdir(cfg.saveDir); end
% timestamp = datestr(now,'yyyymmdd_HHMMSS');
% outName = fullfile(cfg.saveDir, cfg.saveTag + "_" + string(timestamp) + ".mat");
% save(outName, 'agent', 'cfg', 'trainingStats');
% 
% fprintf("\nGespeichert: %s\n", outName);

tail_idx = round(0.7*cfg.maxEpisodes):cfg.maxEpisodes;

% 1) Konsistenz (dimensionslos, vergleichbar zwischen Agenten)
CV = std(trainingStats.EpisodeReward(tail_idx)) ...
   / abs(mean(trainingStats.EpisodeReward(tail_idx)))

% 2) Konvergenz (≈ 0 heißt Plateau erreicht)
drift = mean(diff(trainingStats.AverageReward(tail_idx)))

[value, i] = max(trainingStats.AverageReward)

%% =========================
%  LOKALE RESET-FUNKTION
% =========================
function in = localResetFunctionSpaceKinova(in, cfg)
% Reset-Funktion mit Domain Randomization fuer Sim-to-Real Robustheit.
% Randomisiert Startpose, Gelenkgeschwindigkeit und Basis-Zustand.

nJ = cfg.nJ;

% --- Startpose (aus IK-Loesung) ---
try
    q_start = evalin('base','q_des(1,:).'';');
    if numel(q_start) ~= nJ, error("BadSize"); end
catch
    q_start = zeros(nJ,1);
end

% Stoerung auf Gelenkwinkel (+/- 3 Grad), innerhalb Limits halten
q0 = q_start + deg2rad(3.0) * randn(nJ,1);
q0 = max(min(q0, cfg.qLim_upper), cfg.qLim_lower);

% Kleine Anfangsgeschwindigkeit
dq0 = 0.05 * randn(nJ,1);

% Basis-Zustand: 30% der Episoden nahe-fixiert (simuliert Deployment-Bedingung)
if rand() < 0.3
    base_v0 = zeros(3,1);
    base_w0 = zeros(3,1);
else
    base_v0 = 0.02 * randn(3,1);
    base_w0 = 0.05 * randn(3,1);
end
phi0 = 0;

% --- In Simulink schreiben ---
% in = setVariable(in,'q0', q0);
% in = setVariable(in,'dq0', dq0);
% in = setVariable(in,'base_v0', base_v0);
% in = setVariable(in,'base_w0', base_w0);
% in = setVariable(in,'phi0', phi0);
in = setVariable(in,'reward_init', 0);
in = setVariable(in,'isdone_init', 0);
end
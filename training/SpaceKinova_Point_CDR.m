%% SpaceKinova_Point_CDR.m
% PPO-Training fuer SpaceKinova (POINT-TO-POINT) mit Curriculum Domain
% Randomization (CDR) -- reduzierte Variante.
%
% UEBERTRAGUNG aus dem Trajektorien-CDR-Skript auf die Point-Aufgabe:
%   * Phasen-Loop mit Rekompilierung fuer NON-TUNABLE Parameter
%     (Delay, Reibung), weil diese im Fast-Restart-Modus nicht pro
%     Episode geaendert werden koennen.
%   * q0- und Ziel-Randomisierung bleiben TUNABLE -> pro Episode in ResetFcn.
%
% ENTHALTENE CDR-FEATURES (auf Wunsch reduziert):
%   [CDR-A] Startkonfiguration q0   (tunable,  pro Episode)
%   [CDR-B] Zielpunkt via FK        (tunable,  pro Episode)
%   [CDR-C] Aktuator-Verzoegerung   (non-tunable, pro Sub-Batch fest)
%   [CDR-D] Gelenk-Reibung+Daempfung(non-tunable, pro Sub-Batch fest)
%   [CDR-E] Sensorrauschen          (non-tunable Sigma, pro Phase fest;
%                                    Realisierung pro Zeitschritt via randn
%                                    in einer MATLAB-Function zwischen
%                                    Observation-Vektor und RL_Agent)
%
%   NICHT enthalten (bewusst weggelassen): Masse/Traegheit, Trajektorien.
%
% =========================================================================
%  ERFORDERLICHE SIMULINK-AENDERUNGEN (am Point-Modell)
% =========================================================================
%  Empfehlung: Point-Modell unter NEUEM Namen speichern
%  (SpaceKinova_MotionProfile_point_CDR) und dort folgende Bloecke ergaenzen:
%
%  --- [CDR-C] Aktuator-Verzoegerung ---
%    Integer Delay Block zwischen RL_Agent-Ausgang (dq_cmd) und Joint-Cmd:
%      'Delay'        = From Workspace: act_delay_steps   (Ts_agent-Taktung)
%      Initialzustand = 0 (Nullbefehl)
%    Workspace-Variable: act_delay_steps (Skalar, ganzzahlig, 0-3)
%
%  --- [CDR-D] Gelenk-Daempfung (viskose Reibung) ---
%    Der Simscape Revolute-Primitive bietet unter Internal Mechanics NUR
%    einen Damping Coefficient (viskos, Moment ~ Winkelgeschwindigkeit) und
%    Spring Stiffness. Eine separate Coulomb-Reibung gibt es im Block NICHT
%    -> entfaellt hier. Pro Joint k:
%      Internal Mechanics > Damping Coefficient = joint_damping(k)
%      Spring Stiffness = 0, Equilibrium Position = 0 (unveraendert lassen)
%    Workspace-Variable: joint_damping (7x1), Parameter "Compile-time" (non-tunable).
%
%    !!! WICHTIG -- Motion: Provided by Input !!!
%    Stehen die Joints auf  Actuation > Motion = "Provided by Input"
%    (Torque = Automatically Computed), wird q(t) exakt vorgegeben. Damping/
%    Spring veraendern dann NUR das automatisch berechnete Reaktionsmoment,
%    NICHT die tatsaechliche Bewegung -> die Daempfungs-Randomisierung hat
%    KEINEN Effekt auf Trajektorie/Basis-Reaktion ("Setting ohne Wirkung").
%    Damit Reibung wirkt, muessten die Joints drehmoment-getrieben sein
%    (Torque = Provided by Input) oder die Reibung auf dem Kommandopfad
%    modelliert werden. -> cfg.cdr.enable.friction = false setzen, solange
%    die Joints motion-getrieben sind.
%
%  --- [CDR-E] Sensorrauschen (MATLAB-Function-Block) ---
%    Neuen "MATLAB Function"-Block in die Observation-Leitung einfuegen,
%    DIREKT VOR dem RL_Agent-Block (zwischen Obs-Vektor und Agent):
%
%        function obs_out = addSensorNoise(obs_in, obs_noise_sigma)
%        %#codegen
%        % obs_in:          sauberer Observationsvektor (29x1)
%        % obs_noise_sigma: Per-Komponenten-Std (29x1), pro Phase gesetzt
%            obs_out = obs_in + obs_noise_sigma .* randn(size(obs_in));
%            % --- optionales Clipping auf Obs-Grenzen (auskommentiert) ---
%            % obs_out = max(min(obs_out, obs_hi), obs_lo);
%        end
%
%    - obs_noise_sigma als PARAMETER deklarieren (Ports and Data Manager:
%      Scope = Parameter). Wert kommt aus dem Base-Workspace.
%    - Block-Sample-Time auf -1 (inherited) lassen; der Agent sampelt das
%      Signal mit Ts_agent, randn liefert pro Zeitschritt frisches Rauschen.
%    - obsInfo bleibt unveraendert (Agent sieht weiterhin 29 Werte).
%
% VERSION: SEQUENTIELL (UseParallel = false).
% =========================================================================

clc; clear; close all;
cd(sk_path());   % Modelle laden die Kinova-Meshes relativ zum Repository-Ordner
rng(0,'twister');
clear localResetFunctionCDR;

%% =========================
%  CONFIG (Basis -- aus Point-Skript)
%  =========================
cfg = struct();

% ---- Dateien/Modelle ----
cfg.urdfFile   = sk_path("robot", "SpaceKinova.urdf");
cfg.mdl        = "SpaceKinova_MotionProfile_point_fixed";   % <-- NEUES Modell mit CDR-Bloecken
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

% ---- Nominale physikalische Parameter (fuer Reibung/Daempfung) ----
cfg.joint_damping_nominal = [0.5; 0.5; 0.5; 0.5; 0.3; 0.3; 0.3];

% ---- PPO/Training ----
cfg.maxEpisodes = 3000;
cfg.hiddenUnits = 128;

cfg.saveDir = "savedAgents_spacekinova_point_cdr";
cfg.saveTag = "ppo_spacekinova_point_cdr";

% ---- Logging ----
cfg.logEvery = 25;

%% =========================
%  CDR-CONFIG
%  =========================
cdr = struct();

% --- Anzahl Phasen & Sub-Batches (Rekompilierungen pro Phase) ---
cdr.nPhases             = 4;
cdr.N_samples_per_phase = 4;    % -> 16 Rekompilierungen total

% --- Feature-Enable Flags ---
cdr.enable.actuator_delay = true;    % [CDR-C] non-tunable -> pro Sub-Batch
cdr.enable.friction       = false;   % [CDR-D] AUS: Joints motion-getrieben -> Daempfung ohne Effekt
cdr.enable.sensor_noise   = true;    % [CDR-E] non-tunable Sigma -> pro Phase

% =========================================================================
% [CDR-A/B] q0- & Ziel-Randomisierung (tunable -> ResetFcn) -- pro Phase
%   start_noise_deg       : Std-Abw der q0-Stoerung um q_start_anchor [Grad]
%   target_noise_deg      : Std-Abw der q_target-Stoerung [Grad] (EE-Offset via FK)
%   full_rand_start_frac  : P(vollrandomisiertes q0 in weichen Limits)
% =========================================================================
cdr.phase = struct( ...
    'start_noise_deg',      {  2.0,   8.0,  20.0,  40.0 }, ...
    'target_noise_deg',     {  0.0,   4.0,  12.0,  24.0 }, ...
    'full_rand_start_frac', {  0.0,   0.0,   0.2,   1.0 } );

% =========================================================================
% [CDR-C] AKTUATOR-VERZOEGERUNG (non-tunable -> pro Sub-Batch fest)
% =========================================================================
cdr.delay.steps_min = [0, 0, 0, 1];
cdr.delay.steps_max = [0, 1, 2, 3];

% =========================================================================
% [CDR-D] GELENK-DAEMPFUNG (viskos) (non-tunable -> pro Sub-Batch fest)
%   Revolute-Primitive hat nur Damping Coefficient (keine Coulomb-Reibung).
%   joint_damping = joint_damping_nominal .* damp_factor.
%   ACHTUNG: Wirkt NUR bei drehmoment-getriebenen Joints. Bei
%   Motion = "Provided by Input" ohne Effekt (siehe Header).
% =========================================================================
cdr.damping.factor_std = [0.05, 0.20, 0.45, 0.70];

% =========================================================================
% [CDR-E] SENSORRAUSCHEN (non-tunable Sigma -> pro Phase fest)
%   Per-Komponenten-Std des Observationsvektors (29x1). Realisierung
%   pro Zeitschritt via randn im MATLAB-Function-Block.
%   Reihenfolge muss zum Obs-Vektor passen (siehe Abschnitt 5):
%     [1:3]   EE-Positionsfehler [m]
%     [4:6]   EE-Geschwindigkeitsfehler [m/s]
%     [7:9]   Basis-Lineargeschwindigkeit [m/s]
%     [10:12] Basis-Winkelgeschwindigkeit [rad/s]
%     [13:19] Gelenkpositionen [rad]
%     [20:26] Gelenkgeschwindigkeiten [rad/s]
%     [27:29] Orientierungsfehler [rad]
% =========================================================================
cdr.noise.sigma_nominal = [ ...
    0.003 * ones(3,1); ...   % EE-Positionsfehler
    0.010 * ones(3,1); ...   % EE-Geschwindigkeitsfehler
    0.005 * ones(3,1); ...   % Basis-Lineargeschwindigkeit (IMU)
    0.005 * ones(3,1); ...   % Basis-Winkelgeschwindigkeit (IMU)
    0.002 * ones(7,1); ...   % Gelenkpositionen (Encoder)
    0.010 * ones(7,1); ...   % Gelenkgeschwindigkeiten (differenziert -> lauter)
    0.003 * ones(3,1) ];     % Orientierungsfehler
cdr.noise.phase_scale = [0.0, 0.3, 0.6, 1.0];   % Skalierung von sigma_nominal je Phase

cfg.cdr = cdr;

%% =========================
%  Feature-Uebersicht ausgeben
%  =========================
fprintf('\n========== CDR Feature-Status (POINT) ==========\n');
fprintf('  [CDR-A] q0-Randomisierung        : ON (tunable, pro Episode)\n');
fprintf('  [CDR-B] Ziel-Randomisierung (FK) : ON (tunable, pro Episode)\n');
fb = {'OFF','ON'};
fprintf('  [CDR-C] Aktuator-Delay           : %s (non-tunable, pro Sub-Batch)\n', fb{cdr.enable.actuator_delay+1});
fprintf('  [CDR-D] Gelenk-Daempfung (viskos): %s (non-tunable, pro Sub-Batch)\n', fb{cdr.enable.friction+1});
fprintf('  [CDR-E] Sensorrauschen           : %s (non-tunable Sigma, pro Phase)\n', fb{cdr.enable.sensor_noise+1});
fprintf('  Masse/Traegheit & Trajektorien   : bewusst NICHT enthalten\n');
fprintf('  Fast Restart: ON (innerhalb jeder Phase)\n');
fprintf('  Phasen: %d | Sub-Batches/Phase: %d | Rekompilierungen total: %d\n', ...
    cdr.nPhases, cdr.N_samples_per_phase, cdr.nPhases*cdr.N_samples_per_phase);
fprintf('================================================\n\n');

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

% Initialwerte fuer non-tunable Parameter (vor Phase 1 ueberschrieben)
if cfg.cdr.enable.actuator_delay
    assignin('base','act_delay_steps', 0);
end
if cfg.cdr.enable.friction
    assignin('base','joint_damping', cfg.joint_damping_nominal);
end
if cfg.cdr.enable.sensor_noise
    assignin('base','obs_noise_sigma', zeros(numel(cfg.cdr.noise.sigma_nominal),1));
end

%% =========================
% 2) Konstante Referenz (Nominalziel) -- wird in ResetFcn pro Episode ersetzt
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

% Anker-Objekte in cfg legen, damit ResetFcn ohne evalin auskommt
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

% Default-IC fuer Compile-Time. setVariable in ResetFcn ueberschreibt zur Laufzeit.
assignin('base','q0', cfg.q_start_anchor);

%% =========================
% 4) Simulink Modell laden & konfigurieren
% =========================
load_system(cfg.mdl);
set_param(cfg.mdl, 'FastRestart', 'off');
set_param(cfg.mdl, ...
    'StopTime',   num2str(cfg.T), ...
    'Solver',     'ode14x', ...
    'FixedStep',  num2str(cfg.Ts), ...
    'SolverType', 'Fixed-step');
open_system(cfg.mdl);

%% =========================
% 5) Observation & Action Definition (obsDim = 29, unveraendert)
% =========================
nJ = cfg.nJ;

obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...   % [1:3]   EE-Positionsfehler
    -cfg.eVLim   * ones(3,1); ...   % [4:6]   EE-Geschwindigkeitsfehler
    -cfg.vBLim   * ones(3,1); ...   % [7:9]   Basis-Lineargeschwindigkeit
    -cfg.wBLim   * ones(3,1); ...   % [10:12] Basis-Winkelgeschwindigkeit
     cfg.qLim_lower; ...            % [13:19] Gelenkpositionen
    -cfg.dqLim; ...                 % [20:26] Gelenkgeschwindigkeiten
    -cfg.eOriLim * ones(3,1) ];     % [27:29] Orientierungsfehler

obsHigh = [ ...
     cfg.ePLim   * ones(3,1); ...
     cfg.eVLim   * ones(3,1); ...
     cfg.vBLim   * ones(3,1); ...
     cfg.wBLim   * ones(3,1); ...
     cfg.qLim_upper; ...
     cfg.dqLim; ...
     cfg.eOriLim * ones(3,1) ];

obsInfo = rlNumericSpec([numel(obsLow) 1], ...
    'LowerLimit', obsLow, 'UpperLimit', obsHigh, 'Name', "obs");

actInfo = rlNumericSpec([nJ 1], ...
    'Name', "dq_cmd", ...
    'LowerLimit', -ones(nJ,1), ...
    'UpperLimit',  ones(nJ,1));

% Sicherheitscheck: Sigma-Laenge passt zur Obs-Dimension?
if cfg.cdr.enable.sensor_noise
    assert(numel(cfg.cdr.noise.sigma_nominal) == numel(obsLow), ...
        "sigma_nominal (%d) passt nicht zur Obs-Dimension (%d).", ...
        numel(cfg.cdr.noise.sigma_nominal), numel(obsLow));
end

%% =========================
% 6) RL-Umgebung + ResetFcn
% =========================
% Aktuelle Phase im Base-Workspace, damit ResetFcn die q0/Ziel-Noise-Level kennt.
assignin('base', 'current_cdr_phase', 1);

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
% 8) Sanity Check
% =========================
set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'off');
try
    disp(getActionInfo(agent));
    disp(getObservationInfo(agent));
catch ME
    warning("%s: %s", ME.identifier, ME.message);
end

% CDR-Log initialisieren
cdr_log = struct('episode',[], 'phase',[], 'start_noise_deg',[], ...
    'target_noise_deg',[], 'full_rand_start',[], ...
    'q0_norm_from_anchor',[], 'target_ee_offset_norm',[]);
assignin('base','cdr_log', cdr_log);

%% =========================
% 9) Phasenweises Training mit Fast Restart
% =========================
nPhases     = cfg.cdr.nPhases;
N_sub       = cfg.cdr.N_samples_per_phase;
epsPerPhase = floor(cfg.maxEpisodes / nPhases);

allTrainingStats = cell(nPhases, N_sub);
global_ep_count  = 0;

for phase = 1:nPhases

    fprintf('\n###########################################################\n');
    fprintf('# PHASE %d / %d  (%d Episoden, %d Sub-Batches)\n', ...
        phase, nPhases, epsPerPhase, N_sub);
    fprintf('###########################################################\n');

    assignin('base', 'current_cdr_phase', phase);

    % [CDR-E] Sensorrauschen-Sigma fuer diese Phase (fest ueber alle Sub-Batches)
    if cfg.cdr.enable.sensor_noise
        sigma_phase = cfg.cdr.noise.sigma_nominal * cfg.cdr.noise.phase_scale(phase);
        assignin('base', 'obs_noise_sigma', sigma_phase);
        fprintf('  obs_noise_sigma scale = %.2f  (max sigma = %.4f)\n', ...
            cfg.cdr.noise.phase_scale(phase), max(sigma_phase));
    end

    eps_per_sub  = floor(epsPerPhase / N_sub);
    eps_last_sub = epsPerPhase - (N_sub - 1) * eps_per_sub;

    for sub = 1:N_sub

        fprintf('\n--- Phase %d, Sub-Batch %d/%d ---\n', phase, sub, N_sub);

        % =============================================================
        % Non-tunable Parameter fuer diesen Sub-Batch setzen
        % =============================================================

        % [CDR-C] Aktuator-Verzoegerung
        if cfg.cdr.enable.actuator_delay
            dmin = cfg.cdr.delay.steps_min(phase);
            dmax = cfg.cdr.delay.steps_max(phase);
            if dmax > dmin
                act_delay = randi([dmin, dmax]);
            else
                act_delay = dmin;
            end
            assignin('base', 'act_delay_steps', act_delay);
            fprintf('  act_delay_steps = %d\n', act_delay);
        end

        % [CDR-D] Gelenk-Daempfung (viskos)
        if cfg.cdr.enable.friction
            damp_fac   = max(0.3, 1 + cfg.cdr.damping.factor_std(phase) * randn(cfg.nJ,1));
            joint_damp = cfg.joint_damping_nominal .* damp_fac;
            assignin('base', 'joint_damping', joint_damp);
            fprintf('  joint_damping [min %.3f | max %.3f]\n', ...
                min(joint_damp), max(joint_damp));
        end

        % =============================================================
        % Rekompilieren und Fast Restart aktivieren
        % =============================================================
        set_param(cfg.mdl, 'FastRestart', 'off');
        set_param(cfg.mdl, 'SimulationCommand', 'update');   % Rekompilierung
        set_param(cfg.mdl, 'FastRestart', 'on');
        fprintf('  Modell rekompiliert, Fast Restart ON\n');

        % =============================================================
        % Episoden fuer diesen Sub-Batch
        % =============================================================
        if sub == N_sub
            n_eps = eps_last_sub;
        else
            n_eps = eps_per_sub;
        end

        trainOpts = rlTrainingOptions( ...
            'MaxEpisodes',                n_eps, ...
            'MaxStepsPerEpisode',         floor(cfg.T / cfg.Ts_agent), ...
            'ScoreAveragingWindowLength', 25, ...
            'StopTrainingCriteria',       "AverageReward", ...
            'StopTrainingValue',          1e6, ...        % nicht vorzeitig stoppen
            'Plots',                      "training-progress", ...
            'StopOnError',                "off", ...
            'UseParallel',                false );

        fprintf('  Starte Training: %d Episoden (global %d-%d)\n', ...
            n_eps, global_ep_count + 1, global_ep_count + n_eps);

        allTrainingStats{phase, sub} = train(agent, env, trainOpts);
        global_ep_count = global_ep_count + n_eps;

        set_param(cfg.mdl, 'FastRestart', 'off');
    end
end

set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'on');
fprintf('\n===== Training abgeschlossen. %d Episoden total. =====\n', global_ep_count);

%% =========================
% 10) Save Agent + Stats + CDR-Log
% =========================
if ~exist(cfg.saveDir,'dir'); mkdir(cfg.saveDir); end
stamp = datestr(now,'yyyymmdd_HHMMSS'); %#ok<DATST>
saveFile = fullfile(cfg.saveDir, sprintf("%s_%s.mat", cfg.saveTag, stamp));
cdr_log_final = evalin('base','cdr_log');
cfg_save = rmfield(cfg, 'robot_rbt');  % robot_rbt nicht mitspeichern
save(saveFile, 'agent', 'allTrainingStats', 'cdr_log_final', 'cfg_save');
fprintf("Gespeichert: %s\n", saveFile);

%% =========================
% 11) Post-Training Statistiken
% =========================
allRewards = [];
for ph = 1:nPhases
    for sb = 1:N_sub
        if ~isempty(allTrainingStats{ph, sb})
            allRewards = [allRewards; allTrainingStats{ph, sb}.EpisodeReward]; %#ok<AGROW>
        end
    end
end

if numel(allRewards) > 20
    tail_idx = round(0.7 * numel(allRewards)):numel(allRewards);
    CV    = std(allRewards(tail_idx)) / max(eps, abs(mean(allRewards(tail_idx))));
    drift = mean(diff(allRewards(tail_idx)));
    [bestReward, bestIdx] = max(allRewards);
    fprintf('\n=== Post-Training ===\n');
    fprintf(' CV(tail)=%.3f | drift(tail)=%.4f | BestReward=%.1f @ Ep %d | Total=%d\n', ...
        CV, drift, bestReward, bestIdx, numel(allRewards));
end

%% =========================
% 12) Curriculum-Diagnose-Plot
% =========================
cdr_log_final = evalin('base','cdr_log');
if ~isempty(cdr_log_final.episode)
    figure('Name','CDR Diagnose (Point)');
    subplot(3,1,1);
    plot(cdr_log_final.episode, cdr_log_final.phase, 'k.-'); grid on;
    ylabel('Phase'); title('Curriculum-Phase je Episode');
    subplot(3,1,2);
    plot(cdr_log_final.episode, cdr_log_final.target_ee_offset_norm, 'b.'); grid on;
    ylabel('|EE-Ziel-Offset| [m]'); title('Ziel-Randomisierung (via FK)');
    subplot(3,1,3);
    plot(cdr_log_final.episode, cdr_log_final.q0_norm_from_anchor, 'r.'); grid on;
    ylabel('||q0 - q_{anchor}|| [rad]'); xlabel('Episode');
    title('Start-Randomisierung');
end


%% =========================================================================
%  LOKALE RESET-FUNKTION (nur tunable Features: q0 + Ziel)
%  =========================================================================
function in = localResetFunctionCDR(in, cfg)

nJ    = cfg.nJ;
phase = evalin('base','current_cdr_phase');
pdef  = cfg.cdr.phase(phase);

% ---------------------------------------------------------------
% [CDR-A] q0 randomisieren (um cfg.q_start_anchor)
% ---------------------------------------------------------------
full_rand_start = (rand() < pdef.full_rand_start_frac);
if full_rand_start
    margin = 0.05 * (cfg.qLim_upper - cfg.qLim_lower);
    lo = cfg.qLim_lower + margin;
    hi = cfg.qLim_upper - margin;
    q0 = lo + (hi - lo) .* rand(nJ,1);
else
    sigma_start = deg2rad(pdef.start_noise_deg);
    q0 = cfg.q_start_anchor + sigma_start * randn(nJ,1);
    q0 = max(min(q0, cfg.qLim_upper), cfg.qLim_lower);
end

% ---------------------------------------------------------------
% [CDR-B] Zielpunkt via Joint-Space-Stoerung + FK
% ---------------------------------------------------------------
sigma_tgt = deg2rad(pdef.target_noise_deg);
if sigma_tgt > 0
    q_target_ep = cfg.q_target + sigma_tgt * randn(nJ,1);
    q_target_ep = max(min(q_target_ep, cfg.qLim_upper), cfg.qLim_lower);
else
    q_target_ep = cfg.q_target;
end
T_ee = getTransform(cfg.robot_rbt, q_target_ep.', cfg.eeBodyName);
target_pos_ep = T_ee(1:3,4);

% Referenz-Timeseries
N    = numel(0:cfg.Ts:cfg.T);
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
persistent ep_count_local;
if isempty(ep_count_local), ep_count_local = 0; end
ep_count_local = ep_count_local + 1;

try
    cdr_log = evalin('base','cdr_log');
catch
    cdr_log = struct('episode',[], 'phase',[], 'start_noise_deg',[], ...
        'target_noise_deg',[], 'full_rand_start',[], ...
        'q0_norm_from_anchor',[], 'target_ee_offset_norm',[]);
end
ep_offset_norm = norm(target_pos_ep - cfg.target_pos);
q0_anchor_norm = norm(q0 - cfg.q_start_anchor);
cdr_log.episode(end+1,1)               = ep_count_local;
cdr_log.phase(end+1,1)                 = phase;
cdr_log.start_noise_deg(end+1,1)       = pdef.start_noise_deg;
cdr_log.target_noise_deg(end+1,1)      = pdef.target_noise_deg;
cdr_log.full_rand_start(end+1,1)       = full_rand_start;
cdr_log.q0_norm_from_anchor(end+1,1)   = q0_anchor_norm;
cdr_log.target_ee_offset_norm(end+1,1) = ep_offset_norm;
assignin('base','cdr_log', cdr_log);

if mod(ep_count_local, cfg.logEvery) == 0
    fprintf(['[CDR] ep=%4d  phase=%d  start_sigma=%4.1f deg  full_rand=%d  ' ...
             'tgt_sigma=%4.1f deg  |tgt_off|=%5.3f m  ||dq0||=%5.3f rad\n'], ...
            ep_count_local, phase, pdef.start_noise_deg, full_rand_start, ...
            pdef.target_noise_deg, ep_offset_norm, q0_anchor_norm);
end
end
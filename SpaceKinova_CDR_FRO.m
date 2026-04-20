%% SpaceKinovaDynamic_PPO_CDR.m
% PPO-Training fuer SpaceKinova MIT Curricular Domain Randomization (CDR).
%
% FAST-RESTART-STRATEGIE:
%   Non-tunable Parameter (Simscape Mass/Inertia, Integer Delay) koennen
%   im Fast-Restart-Modus nicht per Episode geaendert werden.
%   -> Loesung: Training in 4 Phasen-Segmente aufgeteilt.
%      Pro Phase: Non-tunables einmal setzen, rekompilieren, Fast Restart ON,
%      dann N Episoden trainieren. Agent wird in-place weitertrainiert.
%
%   Tunable Parameter (Trajektorie via From-Workspace, IC-Bloecke) werden
%   weiterhin per Episode in der ResetFcn randomisiert.
%
% NEU gegenueber Basisversion:
%   [CDR-1] EE-Trajektorie: randomisierte Form/Radius/Geschwindigkeit
%   [CDR-2] Basis-Masse & Traegheit: fest pro Phase (non-tunable)
%   [CDR-3] Aktuator-Verzoegerung: fest pro Phase (non-tunable)
%   [CDR-4] Gelenk-Reibung (viskos + coulomb) & Daempfung: fest pro Phase
%   [CDR-5] Startkonfiguration via Nullraum-Perturbation
%
% Curriculum-Strategie:
%   Phase 1 (Segment 1) : Minimale Randomisierung
%   Phase 2 (Segment 2) : Moderate Stoerungen
%   Phase 3 (Segment 3) : Starke Stoerungen
%   Phase 4 (Segment 4) : Volle Randomisierung
%
% =========================================================================
%  FEATURE-UEBERSICHT & SIMULINK-VORAUSSETZUNGEN
% =========================================================================
%
%  Feature        | Enable-Flag              | Simulink-Aenderung noetig?
%  -----------------------------------------------------------------------
%  CDR-1 Traj.    | cdr.enable.trajectory     | NEIN  (nutzt bestehende From-Workspace-Bloecke)
%  CDR-2 Masse    | cdr.enable.mass_inertia   | JA    (Simscape Solid parametrisieren)
%  CDR-3 Delay    | cdr.enable.actuator_delay | JA    (Integer Delay Block einfuegen)
%  CDR-4 Reibung  | cdr.enable.friction       | JA    (Simscape Joints parametrisieren)
%  CDR-5 q0       | cdr.enable.start_config   | JA    (Initial-Condition-Block fuer q0/dq0)
%  Action History | cdr.useActionHistory      | JA    (Obs-Bus um dq_cmd_prev erweitern)
%
%  -> Features auf false setzen, solange die Simulink-Seite nicht umgebaut ist!
%
% =========================================================================
%  SIMULINK-AENDERUNGEN (Detail-Referenz)
% =========================================================================
%
% --- CDR-2: Basis-Masse & Traegheit ---
%   Simscape Solid Block "Base Cube":
%     Mass             = base_mass_nominal * base_mass_factor     (Skalar)
%     Principal Inertia = I_nominal .* base_inertia_factor        (3x1)
%   Workspace-Variablen:  base_mass_factor (Skalar), base_inertia_factor (3x1)
%
% --- CDR-3: Aktuator-Verzoegerung ---
%   Integer Delay Block zwischen RL_Agent-Ausgang und Joint-Cmd:
%     'Delay' = From Workspace: act_delay_steps  (Ts_agent-Taktung)
%     Initialer Zustand = 0 (Nullbefehl)
%   Workspace-Variable: act_delay_steps (Skalar, ganzzahlig, 0-3)
%
% --- CDR-4: Gelenk-Reibung & Daempfung ---
%   In jedem Simscape Revolute Joint:
%     Friction Torque  = joint_visc_fric .* dq  +  joint_coul_fric .* sign(dq)
%     Damping Coeff.   = joint_damping   (direkt, NICHT als Faktor)
%   Workspace-Variablen: joint_visc_fric (7x1), joint_coul_fric (7x1),
%                         joint_damping (7x1)
%
% --- CDR-5: Startkonfiguration ---
%   Initial-Condition-Bloecke muessen q0 (7x1) und dq0 (7x1)
%   aus dem Workspace lesen.
%
% --- Action History (Obs-Erweiterung) ---
%   Fuege dq_cmd_prev (7x1) als letzten Eintrag in den Obs-Bus ein.
%
% =========================================================================

clc; clear; close all;
rng(0,'twister');

clear localResetFunctionCDR;

%% =========================
%  CONFIG (Basis)
%  =========================
cfg = struct();

% ---- Dateien/Modelle ----
cfg.urdfFile   = "SpaceKinova.urdf";
cfg.mdl        = "SpaceKinova_MotionProfile_CDR";
cfg.agentBlk   = cfg.mdl + "/RL_Agent";
cfg.eeBodyName = "kinova_end_effector_link";

% ---- Freiheitsgrade ----
cfg.nJ = 7;

% ---- Simulations- & Agenten-Zeit ----
cfg.T        = 8.5;
cfg.Ts       = 0.005;
cfg.Ts_agent = 0.025;   % 40 Hz

% ---- Referenz-Trajektorie (Basis-Parameter) ----
cfg.r_nominal  = 0.2;
cfg.center_nom = [0.0, -0.025, 1.687 - cfg.r_nominal];
cfg.omega_nom  = pi / cfg.T;

% ---- Kinova Gen3 7-DOF Gelenkspezifikationen ----
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
cfg.dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];
cfg.safetyFactor = 0.7;
cfg.dq_max     = cfg.safetyFactor * cfg.dqLim;
cfg.tau_max    = [32; 32; 32; 32; 13; 13; 13];

cfg.continuous_joints = [1, 3, 7];

% ---- Sicherheits-Parameter ----
cfg.d_safe   = 0.02;
cfg.dt_agent = cfg.Ts_agent;

% ---- Observation Limits ----
cfg.ePLim   = 0.5;
cfg.eVLim   = 1.0;
cfg.vBLim   = 0.5;
cfg.wBLim   = 1.0;
cfg.eOriLim = pi;

% ---- Nominale physikalische Parameter ----
cfg.base_mass_nominal     = 5.0;                     % [kg]
cfg.base_inertia_nominal  = [0.1; 0.1; 0.1];         % [kg*m^2]
cfg.joint_damping_nominal = [0.5; 0.5; 0.5; 0.5; 0.3; 0.3; 0.3];

% ---- PPO/Training ----
cfg.maxEpisodes    = 1500;
cfg.hiddenUnits    = 128;
cfg.nPhases        = 4;
cfg.epsPerPhase    = floor(cfg.maxEpisodes / cfg.nPhases);  % 500

% ---- Speicherpfade ----
cfg.saveDir = "savedAgents_spacekinova_CDR";
cfg.saveTag = "ppo_spacekinova_CDR";

%% =========================
%  CDR-CONFIG
%  =========================
cdr = struct();

cdr.phase_fractions = [0.0, 0.25, 0.50, 0.75, 1.01];

% =========================================================================
% FEATURE-ENABLE FLAGS
% =========================================================================
cdr.enable.trajectory     = true;   % CDR-1
cdr.enable.mass_inertia   = true;    % CDR-2 (non-tunable -> pro Phase fest)
cdr.enable.actuator_delay = true;    % CDR-3 (non-tunable -> pro Phase fest)
cdr.enable.friction       = true;   % CDR-4 (non-tunable -> pro Phase fest)
cdr.enable.start_config   = false;   % CDR-5 (tunable -> pro Episode)

cdr.useActionHistory = false;

% =========================================================================
% [CDR-1] EE-TRAJEKTORIE
% =========================================================================
cdr.traj.N_lib = 24;

cdr.traj.r_min  = [0.18,  0.14,  0.10,  0.08];
cdr.traj.r_max  = [0.22,  0.24,  0.26,  0.28];

cdr.traj.omega_factor_min = [0.9,  0.7,  0.5,  0.4];
cdr.traj.omega_factor_max = [1.1,  1.3,  1.6,  2.0];

cdr.traj.center_noise_std = [0.0, 0.0, 0.0, 0.0];

cdr.traj.shapes_per_phase = {
    {'halfcircle', 'line_down'};
    {'halfcircle', 'line_down', 'triangle'};
    {'halfcircle', 'line_down', 'triangle', 'l_shape', 's_curve'};
    {'halfcircle', 'line_down', 'triangle', 'l_shape', 's_curve', 'z_shape'};
};

% =========================================================================
% [CDR-2] BASIS-MASSE & TRAEGHEIT  (non-tunable -> 1x pro Phase gesetzt)
% =========================================================================
cdr.mass.factor_std    = [0.03, 0.15, 0.35, 0.65];
%cdr.inertia.factor_std = [0.02, 0.07, 0.12, 0.20];

% Mehrere Samples pro Phase fuer etwas Variation trotz festem Wert:
% Pro Phase werden N_samples Parametersaetze gezogen. Innerhalb der Phase
% wird ein zufaelliger davon verwendet (Rekompilierung pro Sub-Batch).
cdr.mass.N_samples_per_phase = 5;   % -> 3 Rekompilierungen pro Phase = 12 total

% =========================================================================
% [CDR-3] AKTUATOR-VERZOEGERUNG (non-tunable -> 1x pro Phase gesetzt)
% =========================================================================
cdr.delay.steps_min = [0, 0, 0, 1];
cdr.delay.steps_max = [0, 1, 2, 3];

% =========================================================================
% [CDR-4] GELENK-REIBUNG & DAEMPFUNG (non-tunable -> 1x pro Phase gesetzt)
% =========================================================================
cdr.friction.viscous_std  = [0.00, 0.05, 0.12, 0.20];
cdr.friction.coulomb_max  = [0.00, 0.08, 0.18, 0.30];
cdr.damping.factor_std    = [0.05, 0.20, 0.45, 0.70];

% =========================================================================
% [CDR-5] STARTKONFIGURATION (tunable -> pro Episode in ResetFcn)
% =========================================================================
cdr.start.null_amp_base  = 8.0;
cdr.start.null_amp_step  = 5.5;
cdr.start.pos_noise_base = 1.5;
cdr.start.pos_noise_step = 0.5;
cdr.start.dq_noise_base  = 0.02;
cdr.start.dq_noise_step  = 0.02;

cfg.cdr = cdr;

%% =========================
%  Feature-Uebersicht ausgeben
%  =========================
fprintf('\n========== CDR Feature-Status ==========\n');
feat_names  = {'CDR-1 Trajektorie (tunable)', ...
               'CDR-2 Masse/Traegheit (non-tunable, pro Phase)', ...
               'CDR-3 Aktuator-Delay (non-tunable, pro Phase)', ...
               'CDR-4 Reibung/Daempfung (non-tunable, pro Phase)', ...
               'CDR-5 Startkonfiguration (tunable)', ...
               'Action History'};
feat_states = [cdr.enable.trajectory, cdr.enable.mass_inertia, ...
               cdr.enable.actuator_delay, cdr.enable.friction, ...
               cdr.enable.start_config, cdr.useActionHistory];
for fi = 1:numel(feat_names)
    if feat_states(fi)
        fprintf('  [ON]   %s\n', feat_names{fi});
    else
        fprintf('  [OFF]  %s\n', feat_names{fi});
    end
end
fprintf('  Fast Restart: ON (innerhalb jeder Phase)\n');
fprintf('  Rekompilierungen: %d (pro Phase %d Sub-Batches)\n', ...
    cfg.nPhases * cdr.mass.N_samples_per_phase, cdr.mass.N_samples_per_phase);
fprintf('=========================================\n\n');

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

% Initialwerte fuer non-tunable Parameter (werden vor Phase 1 ueberschrieben)
if cfg.cdr.enable.actuator_delay
    assignin('base','act_delay_steps', 0);
end
if cfg.cdr.enable.friction
    assignin('base','joint_visc_fric', zeros(cfg.nJ,1));
    assignin('base','joint_coul_fric', zeros(cfg.nJ,1));
    assignin('base','joint_damping',   cfg.joint_damping_nominal);
end
if cfg.cdr.enable.mass_inertia
    assignin('base','base_mass_factor',    1.0);
    assignin('base','base_inertia_factor', ones(3,1));
end

assignin('base', 'reward_init', 0);
assignin('base', 'isdone_init', 0);
%% =========================
% 2) Robot Import + Trajektorienbibliothek
% =========================
assert(isfile(cfg.urdfFile), "URDF nicht gefunden: %s", cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
if strlength(cfg.eeBodyName) == 0
    cfg.eeBodyName = string(robot_rbt.BodyNames{end});
end
assignin('base','robot_rbt',  robot_rbt);
assignin('base','eeBodyName', cfg.eeBodyName);

t_vec = (0:cfg.Ts:cfg.T)';

if cfg.cdr.enable.trajectory
    % -----------------------------------------------------------------
    %  Trajektorienbibliothek vorberechnen
    % -----------------------------------------------------------------
    fprintf('[CDR] Berechne Trajektorienbibliothek (%d Eintraege)...\n', cfg.cdr.traj.N_lib);

    ik        = inverseKinematics("RigidBodyTree", robot_rbt);
    ikWeights = [1 1 1 0.05 0.05 0.05];

    traj_lib = struct('traj',  cell(cfg.cdr.traj.N_lib, 1), ...
                      'vref',  cell(cfg.cdr.traj.N_lib, 1), ...
                      'q_des', cell(cfg.cdr.traj.N_lib, 1), ...
                      'shape', cell(cfg.cdr.traj.N_lib, 1), ...
                      'phase', cell(cfg.cdr.traj.N_lib, 1));

    entries_per_phase = floor(cfg.cdr.traj.N_lib / 4);
    entry_idx = 0;
    ik_fail_count = 0;

    for ph = 1:4
        n_entries = entries_per_phase + (ph == 4) * (cfg.cdr.traj.N_lib - 4*entries_per_phase);
        shapes_avail = cfg.cdr.traj.shapes_per_phase{ph};

        n_shapes = numel(shapes_avail);
        shape_list = shapes_avail;
        for kk = (n_shapes+1):n_entries
            shape_list{kk} = shapes_avail{randi(n_shapes)};
        end
        shape_list = shape_list(randperm(numel(shape_list)));

        fprintf('  Phase %d shape_list: %s\n', ph, strjoin(shape_list, ', '));

        for k = 1:n_entries
            entry_idx = entry_idx + 1;

            r      = cfg.cdr.traj.r_min(ph) + ...
                     (cfg.cdr.traj.r_max(ph) - cfg.cdr.traj.r_min(ph)) * rand();
            omega  = cfg.omega_nom * ( cfg.cdr.traj.omega_factor_min(ph) + ...
                     (cfg.cdr.traj.omega_factor_max(ph) - cfg.cdr.traj.omega_factor_min(ph)) * rand() );
            cnoise = cfg.cdr.traj.center_noise_std(ph) * randn(1,3);
            center = cfg.center_nom + cnoise;
            shape  = shape_list{k};

            traj_pts = generateTrajectory(shape, t_vec, r, center, omega, cfg.T);
            dt   = mean(diff(t_vec));
            vref = [zeros(1,3); diff(traj_pts) / dt];

            q_des_k = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, traj_pts, t_vec);
            ee_err_max = checkIKQuality(robot_rbt, cfg, q_des_k, traj_pts, t_vec);

            if ee_err_max > 0.02
                fprintf('  *** FAIL #%d (%s, ph=%d): ee_err=%.4f m -> Fallback\n', ...
                    entry_idx, shape, ph, ee_err_max);
                ik_fail_count = ik_fail_count + 1;

                traj_pts = generateTrajectory('halfcircle', t_vec, ...
                    cfg.r_nominal, cfg.center_nom, cfg.omega_nom, cfg.T);
                vref = [zeros(1,3); diff(traj_pts) / dt];
                q_des_k = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, traj_pts, t_vec);
                shape = 'halfcircle';
                ee_err_max = checkIKQuality(robot_rbt, cfg, q_des_k, traj_pts, t_vec);
            end

            traj_lib(entry_idx).traj  = traj_pts;
            traj_lib(entry_idx).vref  = vref;
            traj_lib(entry_idx).q_des = q_des_k;
            traj_lib(entry_idx).shape = shape;
            traj_lib(entry_idx).phase = ph;

            fprintf('  [%02d/%02d] Phase %d | %-12s | r=%.3f m | omega_f=%.2f | ee_err=%.4f m\n', ...
                entry_idx, cfg.cdr.traj.N_lib, ph, shape, r, omega/cfg.omega_nom, ee_err_max);
        end
    end

    if ik_fail_count > 0
        fprintf('[CDR] WARNUNG: %d/%d Trajektorien hatten IK-Probleme (Fallback verwendet).\n', ...
            ik_fail_count, cfg.cdr.traj.N_lib);
    end

    cfg.traj_lib = traj_lib;
    cfg.t_vec    = t_vec;
    assignin('base','traj_lib', traj_lib);

    EE_ref  = timeseries(traj_lib(1).traj, t_vec);
    EE_vref = timeseries(traj_lib(1).vref, t_vec);
    assignin('base','EE_ref',  EE_ref);
    assignin('base','EE_vref', EE_vref);
    assignin('base','q_des',   traj_lib(1).q_des);

    fprintf('[CDR] Bibliothek fertig. %d Trajektorien vorberechnet.\n\n', cfg.cdr.traj.N_lib);

else
    cfg.t_vec = t_vec;
    ik        = inverseKinematics("RigidBodyTree", robot_rbt);
    ikWeights = [1 1 1 0.05 0.05 0.05];

    traj_pts = generateTrajectory('halfcircle', t_vec, ...
        cfg.r_nominal, cfg.center_nom, cfg.omega_nom, cfg.T);
    dt   = mean(diff(t_vec));
    vref = [zeros(1,3); diff(traj_pts) / dt];

    q_des = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, traj_pts, t_vec);

    EE_ref  = timeseries(traj_pts, t_vec);
    EE_vref = timeseries(vref, t_vec);
    assignin('base','EE_ref',  EE_ref);
    assignin('base','EE_vref', EE_vref);
    assignin('base','q_des',   q_des);
    fprintf('[CDR] Trajektorie-Randomisierung deaktiviert. Standard-Halbkreis verwendet.\n\n');
end

%% =========================
% 3) Simulink-Modell laden & konfigurieren
% =========================
load_system(cfg.mdl);
set_param(cfg.mdl, 'FastRestart', 'off');
set_param(cfg.mdl, ...
    'StopTime',   num2str(cfg.T), ...
    'Solver',     'ode14x', ...
    'FixedStep',  num2str(cfg.Ts), ...
    'SolverType', 'Fixed-step');
save_system(cfg.mdl);
open_system(cfg.mdl);

%% =========================
% 4) Observation & Action Definition
% =========================
nJ = cfg.nJ;

obsDim_base = 3 + 3 + 2*nJ + 6 + 3;   % = 29

if cfg.cdr.useActionHistory
    obsDim = obsDim_base + nJ;
    fprintf('[CDR] useActionHistory=true -> obsDim = %d\n', obsDim);
else
    obsDim = obsDim_base;
    fprintf('[CDR] obsDim = %d\n', obsDim);
end

obsLow = [ ...
    -cfg.ePLim   * ones(3,1); ...
    -cfg.eVLim   * ones(3,1); ...
    cfg.qLim_lower; ...
    -cfg.dqLim; ...
    -cfg.vBLim   * ones(3,1); ...
    -cfg.wBLim   * ones(3,1); ...
    -cfg.eOriLim * ones(3,1) ...
    ];

obsHigh = [ ...
    cfg.ePLim   * ones(3,1); ...
    cfg.eVLim   * ones(3,1); ...
    cfg.qLim_upper; ...
    cfg.dqLim; ...
    cfg.vBLim   * ones(3,1); ...
    cfg.wBLim   * ones(3,1); ...
    cfg.eOriLim * ones(3,1) ...
    ];

if cfg.cdr.useActionHistory
    obsLow  = [obsLow;  -ones(nJ,1)];
    obsHigh = [obsHigh;  ones(nJ,1)];
end

obsInfo = rlNumericSpec([obsDim 1], ...
    'LowerLimit', obsLow, ...
    'UpperLimit', obsHigh, ...
    'Name', "obs");

actInfo = rlNumericSpec([nJ 1], ...
    'Name', "dq_cmd", ...
    'LowerLimit', -ones(nJ,1), ...
    'UpperLimit',  ones(nJ,1));

%% =========================
% 5) RL-Umgebung + ResetFcn
% =========================
% Aktuelle Phase wird im Base-Workspace gespeichert, damit die ResetFcn
% weiss in welcher Phase sie ist (fuer CDR-1 und CDR-5 Skalierung).
assignin('base', 'current_cdr_phase', 1);

env = rlSimulinkEnv(cfg.mdl, cfg.agentBlk, obsInfo, actInfo);
env.ResetFcn = @(in) localResetFunctionCDR(in, cfg);

%% =========================
% 6) PPO-Agent
% =========================
initOpts = rlAgentInitializationOptions('NumHiddenUnit', cfg.hiddenUnits);
agent    = rlPPOAgent(obsInfo, actInfo, initOpts);

agent.AgentOptions.SampleTime                       = cfg.Ts_agent;
agent.AgentOptions.ExperienceHorizon                = 600;
agent.AgentOptions.MiniBatchSize                    = 200;
agent.AgentOptions.NumEpoch                         = 10;
agent.AgentOptions.ClipFactor                       = 0.2;

agent.AgentOptions.EntropyLossWeight                = 1e-3;
agent.AgentOptions.ActorOptimizerOptions.LearnRate  = 5.7e-05;
agent.AgentOptions.CriticOptimizerOptions.LearnRate = 1e-03;

assignin('base','agent', agent);

%% =========================
% 7) Sanity Check
% =========================
set_param(bdroot, 'SimMechanicsOpenEditorOnUpdate', 'off');

try
    disp(getActionInfo(agent));
    disp(getObservationInfo(agent));
catch ME
    warning("%s: %s", ME.identifier, ME.message);
end

%% =========================
% 8) Phasenweises Training mit Fast Restart
% =========================
% Aeussere Schleife ueber Phasen, innere Schleife ueber Sub-Batches
% (mehrere Parametersaetze pro Phase fuer Variation).

allTrainingStats = cell(cfg.nPhases, cdr.mass.N_samples_per_phase);
global_ep_count = 0;

for phase = 1:cfg.nPhases
    
    N_sub = cdr.mass.N_samples_per_phase;
    eps_per_sub = floor(cfg.epsPerPhase / N_sub);
    
    % Letzer Sub-Batch bekommt Rest-Episoden
    eps_last_sub = cfg.epsPerPhase - (N_sub - 1) * eps_per_sub;
    
    fprintf('\n');
    fprintf('###########################################################\n');
    fprintf('# PHASE %d / %d  (%d Episoden, %d Sub-Batches)\n', ...
        phase, cfg.nPhases, cfg.epsPerPhase, N_sub);
    fprintf('###########################################################\n');
    
    % Phase im Workspace setzen (fuer ResetFcn)
    assignin('base', 'current_cdr_phase', phase);
    
    for sub = 1:N_sub
        
        % =============================================================
        % Non-tunable Parameter fuer diesen Sub-Batch setzen
        % =============================================================
        fprintf('\n--- Phase %d, Sub-Batch %d/%d ---\n', phase, sub, N_sub);
        
        % [CDR-2] Masse & Traegheit
        if cdr.enable.mass_inertia
            mass_factor    = max(0.5, 1 + cdr.mass.factor_std(phase) * randn());
            geometry_noise = 0.02 * randn(3,1);   % ±2% pro Achse
            inertia_factor = mass_factor * (ones(3,1) + geometry_noise);
            inertia_factor = max(0.5, inertia_factor);
            assignin('base', 'base_mass_factor',    mass_factor);
            assignin('base', 'base_inertia_factor', inertia_factor);
            fprintf('  mass_factor = %.3f, inertia_factor = [%.3f, %.3f, %.3f]\n', ...
                mass_factor, inertia_factor(1), inertia_factor(2), inertia_factor(3));
        end
        
        % [CDR-3] Aktuator-Verzoegerung
        if cdr.enable.actuator_delay
            delay_min = cdr.delay.steps_min(phase);
            delay_max = cdr.delay.steps_max(phase);
            if delay_max > delay_min
                act_delay = randi([delay_min, delay_max]);
            else
                act_delay = delay_min;
            end
            assignin('base', 'act_delay_steps', act_delay);
            fprintf('  act_delay_steps = %d\n', act_delay);
        end
        
        % [CDR-4] Reibung & Daempfung
        if cdr.enable.friction
            joint_visc = cdr.friction.viscous_std(phase) * abs(randn(cfg.nJ,1));
            joint_coul = cdr.friction.coulomb_max(phase) * rand(cfg.nJ,1);
            damp_fac   = max(0.3, 1 + cdr.damping.factor_std(phase) * randn(cfg.nJ,1));
            joint_damp = cfg.joint_damping_nominal .* damp_fac;
            assignin('base', 'joint_visc_fric', joint_visc);
            assignin('base', 'joint_coul_fric', joint_coul);
            assignin('base', 'joint_damping',   joint_damp);
            fprintf('  joint_damp_max = %.3f, joint_damp_min = %.3f\n', max(joint_damp), min(joint_damp));
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
            'StopTrainingValue',           10000, ...
            'Plots',                      "training-progress", ...
            'StopOnError',                "off" ...
        );
        
        fprintf('  Starte Training: %d Episoden (global %d-%d)\n', ...
            n_eps, global_ep_count + 1, global_ep_count + n_eps);
        
        allTrainingStats{phase, sub} = train(agent, env, trainOpts);
        global_ep_count = global_ep_count + n_eps;
        
        % Fast Restart aus fuer naechste Rekompilierung
        set_param(cfg.mdl, 'FastRestart', 'off');
    end
end

fprintf('\n===== Training abgeschlossen. %d Episoden total. =====\n', global_ep_count);

%% =========================
% 9) Post-Training Statistiken
% =========================
% Alle Rewards zusammenfuegen
allRewards = [];
for ph = 1:cfg.nPhases
    for sb = 1:cdr.mass.N_samples_per_phase
        if ~isempty(allTrainingStats{ph, sb})
            allRewards = [allRewards; allTrainingStats{ph, sb}.EpisodeReward]; %#ok<AGROW>
        end
    end
end

if numel(allRewards) > 20
    tail_idx = round(0.7 * numel(allRewards)):numel(allRewards);
    CV    = std(allRewards(tail_idx)) / abs(mean(allRewards(tail_idx)));
    drift = mean(diff(allRewards(tail_idx)));
    [bestReward, bestIdx] = max(allRewards);
    fprintf('\n=== Post-Training ===\n');
    fprintf(' CV=%.3f | drift=%.4f | BestReward=%.1f @ Ep %d\n', ...
        CV, drift, bestReward, bestIdx);
    fprintf(' Total Episodes: %d\n', numel(allRewards));
end


%% =========================================================================
%  LOKALE RESET-FUNKTION (CDR) — nur tunable Features
% =========================================================================
function in = localResetFunctionCDR(in, cfg)
% Nur tunable Features werden hier pro Episode randomisiert.
% Non-tunable Features (CDR-2, CDR-3, CDR-4) werden von der aeusseren
% Phasen-Schleife gesteuert und hier NICHT angefasst.

nJ  = cfg.nJ;
cdr = cfg.cdr;

% Aktuelle Phase aus Workspace lesen
phase = evalin('base', 'current_cdr_phase');

% -----------------------------------------------------------------------
% [CDR-1] EE-Trajektorie aus Bibliothek waehlen (tunable)
% -----------------------------------------------------------------------
if cdr.enable.trajectory
    traj_lib  = evalin('base','traj_lib');
    % Trajektorien bis zur aktuellen Phase verwenden (kumulativ)
    phase_idx = find([traj_lib.phase] <= phase);
    if isempty(phase_idx), phase_idx = 1:numel(traj_lib); end
    sel         = phase_idx(randi(numel(phase_idx)));
    chosen_traj = traj_lib(sel);
    t_vec       = cfg.t_vec;

    assignin('base','EE_ref',  timeseries(chosen_traj.traj, t_vec));
    assignin('base','EE_vref', timeseries(chosen_traj.vref, t_vec));
    assignin('base','q_des',   chosen_traj.q_des);
else
    chosen_traj = struct();
    chosen_traj.q_des = evalin('base','q_des');
    chosen_traj.shape = 'halfcircle';
end

% -----------------------------------------------------------------------
% [CDR-5] Startkonfiguration via Nullraum-Perturbation (tunable)
% -----------------------------------------------------------------------
if cdr.enable.start_config
    try
        q_start_row = chosen_traj.q_des(1,:);
        if numel(q_start_row) ~= nJ, error("BadSize"); end

        robot_rbt = evalin('base','robot_rbt');
        J = geometricJacobian(robot_rbt, q_start_row, cfg.eeBodyName);
        N_basis = null(J);

        null_amp_deg = cdr.start.null_amp_base + (phase-1) * cdr.start.null_amp_step;
        null_amp_rad = deg2rad(null_amp_deg);

        if isempty(N_basis)
            delta_q_null = zeros(nJ,1);
        else
            n_dir = N_basis(:,1);
            alpha  = null_amp_rad * (2*rand() - 1);
            delta_q_null = n_dir * alpha;
        end

        pos_noise_deg = cdr.start.pos_noise_base + (phase-1) * cdr.start.pos_noise_step;
        delta_q_pos   = deg2rad(pos_noise_deg) * randn(nJ,1);

        q0 = q_start_row.' + delta_q_null + delta_q_pos;
        q0 = max(min(q0, 0.8 * cfg.qLim_upper), 0.8 * cfg.qLim_lower);

    catch ME
        warning('NullspaceInit fehlgeschlagen (%s), nutze Fallback.', ME.message);
        q0 = chosen_traj.q_des(1,:).' + deg2rad(5) * randn(nJ,1);
        q0 = max(min(q0, cfg.qLim_upper), cfg.qLim_lower);
    end

    dq_noise = cdr.start.dq_noise_base + (phase-1) * cdr.start.dq_noise_step;
    dq0 = dq_noise * randn(nJ,1);

    in = setVariable(in, 'q0',  q0);
    in = setVariable(in, 'dq0', dq0);
end

% -----------------------------------------------------------------------
% Reward/Done Reset (immer aktiv)
% -----------------------------------------------------------------------
in = setVariable(in, 'reward_init', 0);
in = setVariable(in, 'isdone_init', 0);

% -----------------------------------------------------------------------
% Debug-Log (alle 50 Episoden)
% -----------------------------------------------------------------------
persistent ep_count_local;
if isempty(ep_count_local), ep_count_local = 0; end
ep_count_local = ep_count_local + 1;

if mod(ep_count_local, 50) == 1
    fprintf('[CDR ep=%4d] Phase=%d | shape=%-12s\n', ...
        ep_count_local, phase, chosen_traj.shape);
end
end  % localResetFunctionCDR


%% =========================================================================
%  HILFSFUNKTION: Trajektorie generieren
% =========================================================================
function pts = generateTrajectory(shape, t_vec, r, center, omega, T_ep)

N   = numel(t_vec);
pts = zeros(N, 3);

cx = center(1);
cy = center(2);
cz = center(3);
pts(:,2) = cy;

s = t_vec(:) / T_ep;

switch lower(shape)

    case 'halfcircle'
        pts(:,1) = cx + r * sin(omega * t_vec);
        pts(:,3) = cz + r * cos(omega * t_vec);

    case 'line_down'
        pts(:,1) = cx;
        pts(:,3) = cz + r - 2*r*s;

    case 'triangle'
        P1 = [cx,     cy, cz + r];
        P2 = [cx + r, cy, cz    ];
        P3 = [cx,     cy, cz - r];
        T_half = T_ep / 2;
        for ki = 1:N
            if t_vec(ki) <= T_half
                f = t_vec(ki) / T_half;
                pts(ki,:) = (1-f)*P1 + f*P2;
            else
                f = (t_vec(ki) - T_half) / T_half;
                pts(ki,:) = (1-f)*P2 + f*P3;
            end
        end

    case 's_curve'
        pts(:,1) = cx + r * sin(2*pi*s);
        pts(:,3) = cz + r - 2*r*s;

    case 'l_shape'
        P1 = [cx,     cy, cz + r];
        P2 = [cx,     cy, cz - r];
        P3 = [cx + r, cy, cz - r];
        T_half = T_ep / 2;
        for ki = 1:N
            if t_vec(ki) <= T_half
                f = t_vec(ki) / T_half;
                pts(ki,:) = (1-f)*P1 + f*P2;
            else
                f = (t_vec(ki) - T_half) / T_half;
                pts(ki,:) = (1-f)*P2 + f*P3;
            end
        end

    case 'z_shape'
        P1 = [cx,         cy, cz + r];
        P2 = [cx + r*0.5, cy, cz + r*0.3];
        P3 = [cx - r*0.5, cy, cz - r*0.3];
        P4 = [cx + r*0.5, cy, cz - r];
        T3 = T_ep / 3;
        for ki = 1:N
            if t_vec(ki) <= T3
                f = t_vec(ki) / T3;
                pts(ki,:) = (1-f)*P1 + f*P2;
            elseif t_vec(ki) <= 2*T3
                f = (t_vec(ki) - T3) / T3;
                pts(ki,:) = (1-f)*P2 + f*P3;
            else
                f = (t_vec(ki) - 2*T3) / T3;
                pts(ki,:) = (1-f)*P3 + f*P4;
            end
        end

    otherwise
        warning('generateTrajectory: Unbekannte Form "%s", nutze halfcircle.', shape);
        pts(:,1) = cx + r * sin(omega * t_vec);
        pts(:,3) = cz + r * cos(omega * t_vec);
end
end  % generateTrajectory


%% =========================================================================
%  HILFSFUNKTION: IK mit Wrapping der continuous Joints
% =========================================================================
function q_des = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, traj_pts, t_vec)

nJ    = cfg.nJ;
N     = numel(t_vec);
q_des = zeros(N, nJ);
qSeed = homeConfiguration(robot_rbt);
R0    = eye(3);

warnState = warning('off', 'all');
for ki = 1:N
    Tgoal = [R0, traj_pts(ki,:).'; 0 0 0 1];
    [qSol, ~] = ik(cfg.eeBodyName, Tgoal, ikWeights, qSeed);
    q_des(ki,:) = qSol(1:nJ);
    qSeed       = qSol;
end
warning(warnState);

end  % solveIKWithWrapping


%% =========================================================================
%  HILFSFUNKTION: IK-Qualitaet pruefen
% =========================================================================
function ee_err_max = checkIKQuality(robot_rbt, cfg, q_des, traj_pts, t_vec)

N = numel(t_vec);
ee_actual = zeros(N, 3);
for ki = 1:N
    T_fk = getTransform(robot_rbt, q_des(ki,:), cfg.eeBodyName);
    ee_actual(ki,:) = T_fk(1:3,4)';
end
ee_err_max = max(vecnorm(ee_actual - traj_pts, 2, 2));

end  % checkIKQuality
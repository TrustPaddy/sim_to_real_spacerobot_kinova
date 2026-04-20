%% SpaceKinovaDynamic_PPO_CDR.m
% PPO-Training fuer SpaceKinova MIT Curricular Domain Randomization (CDR).
%
% NEU gegenueber Basisversion:
%   [CDR-1] EE-Trajektorie: randomisierte Form/Radius/Geschwindigkeit
%           (Vorberechnung einer Bibliothek, kein IK-Overhead per Episode)
%   [CDR-2] Basis-Masse & Traegheit: skalierbare Stoerung per Phase
%   [CDR-3] Aktuator-Verzoegerung: 0-3 Steps, zugewiesen per Phase
%   [CDR-4] Gelenk-Reibung (viskos + coulomb) & Daempfung per Phase
%   [CDR-5] Startkonfiguration via Nullraum-Perturbation
%
% Curriculum-Strategie:
%   Phase 1 (0-25%)   : Minimale Randomisierung -> stabiles Basislernverhalten
%   Phase 2 (25-50%)  : Moderate Stoerungen -> Robustheit aufbauen
%   Phase 3 (50-75%)  : Starke Stoerungen
%   Phase 4 (75-100%) : Volle Randomisierung, alle Sim-to-Real Effekte aktiv
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
%     Sonst werden Variablen gesetzt, die kein Block liest = kein Effekt.
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
%   aus dem Workspace lesen. Ohne diese Anbindung hat die Reset-
%   Randomisierung keinen Effekt.
%
% --- Action History (Obs-Erweiterung) ---
%   Fuege dq_cmd_prev (7x1) als letzten Eintrag in den Obs-Bus ein.
%   Das Signal ist die Agent-Ausgabe (normalisiert [-1,+1]) mit einem
%   Schritt Verzoegerung (Unit Delay auf dq_cmd).
%
% =========================================================================
%  HINWEIS ZUR REWARD-FUNKTION
% =========================================================================
% Die bestehende Reward-Funktion in Simulink sollte ohne Aenderungen
% funktionieren, da sie vermutlich auf dem Tracking-Error (||ee - ee_ref||)
% basiert, und EE_ref per Episode korrekt gesetzt wird.
%
% Falls der Reward bei schwierigeren Trajektorien (Phase 3/4) stark
% einbricht und das Training instabil wird, gibt es zwei Optionen:
%   1) Die Curriculum-Phase-Grenzen anpassen (spaeterer Uebergang)
%   2) Den Reward pro Episode durch die Trajektorie-Geschwindigkeit
%      normalisieren (advanced, nur wenn noetig)
% =========================================================================

clc; clear; close all;
rng(0,'twister');

% Persistent-Zaehler in ResetFcn zuruecksetzen (damit Curriculum bei
% Skript-Neustart sauber bei Phase 1 beginnt)
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

% ---- Referenz-Trajektorie (Basis-Parameter, werden pro Episode randomisiert) ----
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

% Continuous Joints (J1, J3, J7): haben in der URDF keine festen Limits,
% Software-Limit ±2π. Der IK-Solver kann Werte ausserhalb liefern, die
% per wrapToPi normalisiert werden muessen.
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

% ---- Nominale physikalische Parameter (fuer CDR-Skalierung) ----
cfg.base_mass_nominal     = 5.0;                     % [kg]   (an Simscape anpassen!)
cfg.base_inertia_nominal  = [0.1; 0.1; 0.1];         % [kg*m^2]
cfg.joint_damping_nominal = [0.5; 0.5; 0.5; 0.5; 0.3; 0.3; 0.3]; % [Nm*s/rad]

% ---- PPO/Training ----
cfg.maxEpisodes = 3000;
cfg.hiddenUnits = 128;

% ---- Speicherpfade ----
cfg.saveDir = "savedAgents_spacekinova_CDR";
cfg.saveTag = "ppo_spacekinova_CDR";

%% =========================
%  CDR-CONFIG (Curricular Domain Randomization)
%  =========================
cdr = struct();

% --- Curriculum-Phasen-Grenzen (Anteil der Gesamtepisoden) ---
cdr.phase_fractions = [0.0, 0.25, 0.50, 0.75, 1.01]; % 4 Phasen

% =========================================================================
% FEATURE-ENABLE FLAGS
% Auf false setzen, wenn die entsprechende Simulink-Aenderung NICHT
% umgesetzt ist. Sonst werden Variablen geschrieben, die nichts bewirken.
% =========================================================================
cdr.enable.trajectory     = false;    % CDR-1: funktioniert sofort (From Workspace)
cdr.enable.mass_inertia   = false;   % CDR-2: braucht Simscape-Parametrisierung
cdr.enable.actuator_delay = true;   % CDR-3: braucht Integer Delay Block
cdr.enable.friction       = false;   % CDR-4: braucht Simscape-Joint-Anpassung
cdr.enable.start_config   = false;   % CDR-5: braucht IC-Block fuer q0/dq0

% Observation-Erweiterung: letzte Aktion als Hilfsinfo fuer Delay-Robustheit
% -> Nur aktivieren, wenn Simulink Obs-Bus um dq_cmd_prev(7x1) erweitert!
cdr.useActionHistory = false;

% =========================================================================
% [CDR-1] EE-TRAJEKTORIE
% =========================================================================
cdr.traj.N_lib = 24;   % 6 Trajektorien pro Phase x 4 Phasen

% Radius-Bereich [m]  pro Phase:  Phase1  Phase2  Phase3  Phase4
% Konservativ gehalten damit auch l_shape/z_shape im Arbeitsraum bleiben
% (diese Formen haben groessere Ausdehnung als der Halbkreis bei gleichem r)
cdr.traj.r_min  = [0.18,  0.14,  0.10,  0.08];
cdr.traj.r_max  = [0.22,  0.24,  0.26,  0.28];

% Omega-Faktor (relativ zu omega_nom):
cdr.traj.omega_factor_min = [0.9,  0.7,  0.5,  0.4];
cdr.traj.omega_factor_max = [1.1,  1.3,  1.6,  2.0];

% Mittelpunkt-Rauschen [m] (Gauss-Std pro Achse):
% Auf 0 gesetzt: Formenvielfalt + Radius/Omega-Variation reichen fuer
% Generalisierung. Center-Noise verschiebt Eckpunkte aus dem Arbeitsraum
% und verursacht unnoetige IK-Failures. Kann spaeter aktiviert werden
% wenn CDR-5 (q0-Anpassung) implementiert ist.
cdr.traj.center_noise_std = [0.0, 0.0, 0.0, 0.0];

% ---- 6 Formen, maximal unterschiedliche Bewegungsmuster ----
% Alle starten am gleichen Punkt: [cx, cy, cz + r]
%
%   halfcircle : glatter Bogen (gleichmaessige Kruemmung)
%   line_down  : reine Gerade nach unten (keine Kruemmung)
%   triangle   : scharfer V-Knick (abrupter Richtungswechsel)
%   s_curve    : Kruemmungsumkehr (Sinuskurve, links-rechts beim Absinken)
%   l_shape    : 90-Grad-Ecke (vertikal -> horizontal)
%   z_shape    : 3 Diagonalsegmente mit 2 scharfen Richtungswechseln
%
% Erlaubte Formen pro Phase (kumulativ, einfach -> schwer)
cdr.traj.shapes_per_phase = {
    {'halfcircle', 'line_down'};                                             % Phase 1
    {'halfcircle', 'line_down', 'triangle'};                                 % Phase 2
    {'halfcircle', 'line_down', 'triangle', 'l_shape', 's_curve'};           % Phase 3
    {'halfcircle', 'line_down', 'triangle', 'l_shape', 's_curve', 'z_shape'};% Phase 4
};

% =========================================================================
% [CDR-2] BASIS-MASSE & TRAEGHEIT  (nur aktiv wenn enable.mass_inertia)
% =========================================================================
cdr.mass.factor_std    = [0.03, 0.08, 0.15, 0.25];   % Gauss-Std
cdr.inertia.factor_std = [0.02, 0.07, 0.12, 0.20];

% =========================================================================
% [CDR-3] AKTUATOR-VERZOEGERUNG     (nur aktiv wenn enable.actuator_delay)
% =========================================================================
cdr.delay.steps_min = [0, 0, 0, 1];
cdr.delay.steps_max = [0, 1, 2, 3];

% =========================================================================
% [CDR-4] GELENK-REIBUNG & DAEMPFUNG (nur aktiv wenn enable.friction)
% =========================================================================
cdr.friction.viscous_std  = [0.00, 0.05, 0.12, 0.20];
cdr.friction.coulomb_max  = [0.00, 0.08, 0.18, 0.30];
cdr.damping.factor_std    = [0.02, 0.07, 0.15, 0.25];

% =========================================================================
% [CDR-5] STARTKONFIGURATION        (nur aktiv wenn enable.start_config)
% =========================================================================
cdr.start.null_amp_base  = 8.0;     % [deg] Nullraum-Amplitude Phase 1
cdr.start.null_amp_step  = 5.5;     % [deg] Zunahme pro Phase
cdr.start.pos_noise_base = 1.5;     % [deg] Positions-Stoerung Phase 1
cdr.start.pos_noise_step = 0.5;     % [deg] Zunahme pro Phase
cdr.start.dq_noise_base  = 0.02;    % [rad/s] Geschwindigkeits-Rauschen Phase 1
cdr.start.dq_noise_step  = 0.02;    % [rad/s] Zunahme pro Phase

cfg.cdr = cdr;

%% =========================
%  Feature-Uebersicht ausgeben
%  =========================
fprintf('\n========== CDR Feature-Status ==========\n');
feat_names  = {'CDR-1 Trajektorie', 'CDR-2 Masse/Traegheit', ...
               'CDR-3 Aktuator-Delay', 'CDR-4 Reibung/Daempfung', ...
               'CDR-5 Startkonfiguration', 'Action History'};
feat_states = [cdr.enable.trajectory, cdr.enable.mass_inertia, ...
               cdr.enable.actuator_delay, cdr.enable.friction, ...
               cdr.enable.start_config, cdr.useActionHistory];
for fi = 1:numel(feat_names)
    if feat_states(fi)
        fprintf('  [ON]   %s\n', feat_names{fi});
    else
        fprintf('  [OFF]  %s  (Simulink-Aenderung noetig)\n', feat_names{fi});
    end
end
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

% CDR-Initialwerte nur setzen, wenn das Feature aktiv ist
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

        % Shape-Liste: erst jede Form 1x, dann zufaellig auffuellen
        n_shapes = numel(shapes_avail);
        shape_list = shapes_avail;   % garantiert jede Form einmal
        for kk = (n_shapes+1):n_entries
            shape_list{kk} = shapes_avail{randi(n_shapes)};
        end
        % Reihenfolge mischen damit die garantierten nicht immer zuerst kommen
        shape_list = shape_list(randperm(numel(shape_list)));

        fprintf('  Phase %d shape_list: %s\n', ph, strjoin(shape_list, ', '));

        for k = 1:n_entries
            entry_idx = entry_idx + 1;

            % Parameter randomisieren
            r      = cfg.cdr.traj.r_min(ph) + ...
                     (cfg.cdr.traj.r_max(ph) - cfg.cdr.traj.r_min(ph)) * rand();
            omega  = cfg.omega_nom * ( cfg.cdr.traj.omega_factor_min(ph) + ...
                     (cfg.cdr.traj.omega_factor_max(ph) - cfg.cdr.traj.omega_factor_min(ph)) * rand() );
            cnoise = cfg.cdr.traj.center_noise_std(ph) * randn(1,3);
            center = cfg.center_nom + cnoise;
            shape  = shape_list{k};

            % Trajektorie generieren
            traj_pts = generateTrajectory(shape, t_vec, r, center, omega, cfg.T);

            % Geschwindigkeitsreferenz (finite Differenzen)
            dt   = mean(diff(t_vec));
            vref = [zeros(1,3); diff(traj_pts) / dt];

            % IK loesen mit Normalisierung der continuous Joints
            q_des_k = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, ...
                          traj_pts, t_vec);

            % ---- IK-Qualitaetspruefung (FK-Vergleich nach Wrapping) ----
            ee_err_max = checkIKQuality(robot_rbt, cfg, q_des_k, traj_pts, t_vec);

            if ee_err_max > 0.02   % 2 cm EE-Toleranz (einziges Kriterium)
                fprintf('  *** FAIL #%d (%s, ph=%d): ee_err=%.4f m -> Fallback\n', ...
                    entry_idx, shape, ph, ee_err_max);
                ik_fail_count = ik_fail_count + 1;

                % Fallback: Halbkreis mit Nominal-Parametern
                traj_pts = generateTrajectory('halfcircle', t_vec, ...
                    cfg.r_nominal, cfg.center_nom, cfg.omega_nom, cfg.T);
                vref = [zeros(1,3); diff(traj_pts) / dt];
                q_des_k = solveIKWithWrapping(robot_rbt, cfg, ik, ikWeights, ...
                              traj_pts, t_vec);
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
        fprintf('       -> Ggf. center_noise_std oder r-Bereiche in spaeteren Phasen reduzieren.\n');
    end

    cfg.traj_lib = traj_lib;
    cfg.t_vec    = t_vec;
    assignin('base','traj_lib', traj_lib);

    % Starttrajektorie setzen (Phase 1, Eintrag 1)
    EE_ref  = timeseries(traj_lib(1).traj, t_vec);
    EE_vref = timeseries(traj_lib(1).vref, t_vec);
    assignin('base','EE_ref',  EE_ref);
    assignin('base','EE_vref', EE_vref);
    assignin('base','q_des',   traj_lib(1).q_des);

    fprintf('[CDR] Bibliothek fertig. %d Trajektorien vorberechnet.\n\n', cfg.cdr.traj.N_lib);

else
    % -----------------------------------------------------------------
    %  Keine Trajektorie-Randomisierung: Standard-Halbkreis wie Original
    % -----------------------------------------------------------------
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

% Basis-Obs: [ep(3); ev(3); q(nJ); dq(nJ); vbase(3); wbase(3); e_ori(3)]
obsDim_base = 3 + 3 + 2*nJ + 6 + 3;   % = 29

if cfg.cdr.useActionHistory
    obsDim = obsDim_base + nJ;           % 29 + 7 = 36
    fprintf('[CDR] useActionHistory=true -> obsDim = %d (Simulink Obs-Bus anpassen!)\n', obsDim);
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
    % Agent-Ausgabe ist normalisiert auf [-1, +1]
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
% 8) Training Options + Train
% =========================
trainOpts = rlTrainingOptions( ...
    'MaxEpisodes',                cfg.maxEpisodes, ...
    'MaxStepsPerEpisode',         floor(cfg.T / cfg.Ts_agent), ...
    'ScoreAveragingWindowLength', 25, ...
    'StopTrainingCriteria',       "AverageReward", ...
    'StopTrainingValue',           10000, ...
    'Plots',                      "training-progress", ...
    'StopOnError',                "off" ...
);

% Besten Agenten automatisch speichern
% if ~isfolder(cfg.saveDir), mkdir(cfg.saveDir); end
% saveDirRun = fullfile(cfg.saveDir, cfg.saveTag + "_" + string(datestr(now,'yyyymmdd_HHMMSS')));
% trainOpts.SaveAgentCriteria  = "EpisodeReward";
% trainOpts.SaveAgentValue     = -inf;
% trainOpts.SaveAgentDirectory = saveDirRun;


trainingStats = train(agent, env, trainOpts);

%% =========================
% 9) Post-Training Statistiken
% =========================
tail_idx = round(0.7*cfg.maxEpisodes):cfg.maxEpisodes;
CV    = std(trainingStats.EpisodeReward(tail_idx)) ...
      / abs(mean(trainingStats.EpisodeReward(tail_idx)));
drift = mean(diff(trainingStats.AverageReward(tail_idx)));
[value, i] = max(trainingStats.AverageReward);
fprintf('\n=== Post-Training ===\n CV=%.3f | drift=%.4f | BestAvgReward=%.1f @ Ep %d\n', ...
    CV, drift, value, i);


%% =========================================================================
%  LOKALE RESET-FUNKTION (CDR)
% =========================================================================
function in = localResetFunctionCDR(in, cfg)
% Curricular Domain Randomization Reset.
%
% Nur aktivierte Features werden geschrieben.
% Nicht-aktivierte Features werden uebersprungen.

nJ  = cfg.nJ;
cdr = cfg.cdr;

% -----------------------------------------------------------------------
% [A] Curriculum-Phase bestimmen
% -----------------------------------------------------------------------
persistent ep_count;
if isempty(ep_count), ep_count = 0; end
ep_count = ep_count + 1;

frac  = min((ep_count - 1) / max(cfg.maxEpisodes - 1, 1), 1.0);
phase = find(frac >= cdr.phase_fractions(1:end-1) & ...
             frac <  cdr.phase_fractions(2:end), 1);
if isempty(phase), phase = 4; end

% -----------------------------------------------------------------------
% [CDR-1] EE-Trajektorie aus Bibliothek waehlen
% -----------------------------------------------------------------------
if cdr.enable.trajectory
    traj_lib  = evalin('base','traj_lib');
    phase_idx = find([traj_lib.phase] == phase);
    if isempty(phase_idx), phase_idx = 1:numel(traj_lib); end
    sel         = phase_idx(randi(numel(phase_idx)));
    chosen_traj = traj_lib(sel);
    t_vec       = cfg.t_vec;

    assignin('base','EE_ref',  timeseries(chosen_traj.traj, t_vec));
    assignin('base','EE_vref', timeseries(chosen_traj.vref, t_vec));
    assignin('base','q_des',   chosen_traj.q_des);
else
    % q_des trotzdem laden (fuer CDR-5 Nullraum-Berechnung)
    chosen_traj = struct();
    chosen_traj.q_des = evalin('base','q_des');
    chosen_traj.shape = 'halfcircle';
end

% -----------------------------------------------------------------------
% [CDR-2] Basis-Masse & Traegheit
% -----------------------------------------------------------------------
mass_factor = 1.0;   % Default fuer Debug-Log
if cdr.enable.mass_inertia
    mass_factor    = max(0.5, 1 + cdr.mass.factor_std(phase) * randn());
    inertia_factor = max(0.5, ones(3,1) + cdr.inertia.factor_std(phase) * randn(3,1));
    in = setVariable(in, 'base_mass_factor',    mass_factor);
    in = setVariable(in, 'base_inertia_factor', inertia_factor);
end

% -----------------------------------------------------------------------
% [CDR-3] Aktuator-Verzoegerung
% -----------------------------------------------------------------------
act_delay = 0;       % Default fuer Debug-Log
if cdr.enable.actuator_delay
    delay_min = cdr.delay.steps_min(phase);
    delay_max = cdr.delay.steps_max(phase);
    if delay_max > delay_min
        act_delay = randi([delay_min, delay_max]);
    else
        act_delay = delay_min;
    end
    in = setVariable(in, 'act_delay_steps', act_delay);
end

% -----------------------------------------------------------------------
% [CDR-4] Gelenk-Reibung & Daempfung
% -----------------------------------------------------------------------
joint_visc = zeros(nJ,1);   % Default fuer Debug-Log
if cdr.enable.friction
    joint_visc  = cdr.friction.viscous_std(phase) * abs(randn(nJ,1));
    joint_coul  = cdr.friction.coulomb_max(phase) * rand(nJ ...
        ,1);
    damp_factor = max(0.3, 1 + cdr.damping.factor_std(phase) * randn(nJ,1));
    joint_damp  = cfg.joint_damping_nominal .* damp_factor;
    in = setVariable(in, 'joint_visc_fric', joint_visc);
    in = setVariable(in, 'joint_coul_fric', joint_coul);
    in = setVariable(in, 'joint_damping',   joint_damp);
end

% -----------------------------------------------------------------------
% [CDR-5] Startkonfiguration via Nullraum-Perturbation + dq0
% -----------------------------------------------------------------------
null_amp_deg = 0;    % Default fuer Debug-Log
if cdr.enable.start_config
    try
        q_start_row = chosen_traj.q_des(1,:);   % 1 x nJ
        if numel(q_start_row) ~= nJ, error("BadSize"); end

        robot_rbt = evalin('base','robot_rbt');
        J = geometricJacobian(robot_rbt, q_start_row, cfg.eeBodyName);

        % Nullraum-Basis: absichern gegen leere / mehrspaltige Ergebnisse
        N_basis = null(J);

        null_amp_deg = cdr.start.null_amp_base + (phase-1) * cdr.start.null_amp_step;
        null_amp_rad = deg2rad(null_amp_deg);

        if isempty(N_basis)
            delta_q_null = zeros(nJ,1);
        else
            n_dir = N_basis(:,1);        % nur erste Nullraum-Richtung
            alpha  = null_amp_rad * (2*rand() - 1);
            delta_q_null = n_dir * alpha;   % garantiert 7x1
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

    % Anfangsgeschwindigkeit (phasenabhaengig)
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
if mod(ep_count, 50) == 1
    fprintf('[CDR ep=%4d] Ph=%d | %-12s | delay=%d | mass=%.2f | visc_max=%.3f | null_amp=%.1f deg\n', ...
        ep_count, phase, chosen_traj.shape, act_delay, ...
        mass_factor, max(joint_visc), null_amp_deg);
end
end  % localResetFunctionCDR


%% =========================================================================
%  HILFSFUNKTION: Trajektorie generieren
% =========================================================================
function pts = generateTrajectory(shape, t_vec, r, center, omega, T_ep)
% Gibt (N x 3) Kartesische Punkte zurueck.
% Alle 6 Formen starten am gleichen Punkt: [cx, cy, cz + r]
%
%   shape  : 'halfcircle' | 'line_down' | 'triangle' | 's_curve' | 'l_shape' | 'z_shape'
%   t_vec  : Zeitvektor (N x 1) [s]
%   r      : Charakteristischer Radius [m]
%   center : Mittelpunkt [1x3]
%   omega  : Winkelgeschwindigkeit [rad/s]
%   T_ep   : Episodendauer [s]

N   = numel(t_vec);
pts = zeros(N, 3);

cx = center(1);
cy = center(2);
cz = center(3);
pts(:,2) = cy;   % y konstant (x-z-Ebene)

s = t_vec(:) / T_ep;   % normalisierter Fortschritt 0 -> 1

switch lower(shape)

    case 'halfcircle'
    % Glatter Bogen von oben nach unten (gleichmaessige Kruemmung)
        pts(:,1) = cx + r * sin(omega * t_vec);
        pts(:,3) = cz + r * cos(omega * t_vec);

    case 'line_down'
    % Gerade Linie von oben nach unten (keine Kruemmung)
        pts(:,1) = cx;
        pts(:,3) = cz + r - 2*r*s;

    case 'triangle'
    % V-Knick: oben -> rechts-mitte -> unten (1 scharfer Richtungswechsel)
        P1 = [cx,     cy, cz + r];
        P2 = [cx - r, cy, cz    ];
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
    % Kruemmungsumkehr: Sinuskurve links-rechts beim Absinken
        pts(:,1) = cx - r * sin(2*pi*s);
        pts(:,3) = cz - r + 2*r*s;

    case 'l_shape'
    % 90-Grad-Ecke: vertikal runter -> horizontal nach rechts
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
    % 3 Diagonalsegmente: rechts-runter, links-runter, rechts-runter
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
% Loest IK fuer alle Zeitschritte — identisch zum Originalskript.
%
% Kein Wrapping oder Clamping waehrend oder nach der Schleife.
% Die continuous Joints (J1,J3,J7) koennen Werte > 2*pi haben,
% was kinematisch korrekt ist. Der Limit-Check in checkIKQuality
% beruecksichtigt das.

nJ    = cfg.nJ;
N     = numel(t_vec);
q_des = zeros(N, nJ);
qSeed = homeConfiguration(robot_rbt);
R0    = eye(3);

warnState = warning('off', 'all');  % IK-Solver warnt bei Seeds ausserhalb URDF-Limits
for ki = 1:N
    Tgoal = [R0, traj_pts(ki,:).'; 0 0 0 1];
    [qSol, ~] = ik(cfg.eeBodyName, Tgoal, ikWeights, qSeed);
    q_des(ki,:) = qSol(1:nJ);
    qSeed       = qSol;
end
warning(warnState);

end  % solveIKWithWrapping


%% =========================================================================
%  HILFSFUNKTION: IK-Qualitaet pruefen (EE-Fehler + Gelenkgrenzen)
% =========================================================================
function ee_err_max = checkIKQuality(robot_rbt, cfg, q_des, traj_pts, t_vec)
% Prueft ob die IK-Loesung kinematisch korrekt ist:
% Einziges Kriterium: EE-Position stimmt mit Soll ueberein (FK-Vergleich).
% Gelenkgrenzen werden NICHT geprueft — der IK-Solver ueberschreitet
% Limits manchmal minimal, was fuer die Simulation unkritisch ist
% (Simscape erzwingt Limits intern).

N = numel(t_vec);
ee_actual = zeros(N, 3);
for ki = 1:N
    T_fk = getTransform(robot_rbt, q_des(ki,:), cfg.eeBodyName);
    ee_actual(ki,:) = T_fk(1:3,4)';
end
ee_err_max = max(vecnorm(ee_actual - traj_pts, 2, 2));

end  % checkIKQuality
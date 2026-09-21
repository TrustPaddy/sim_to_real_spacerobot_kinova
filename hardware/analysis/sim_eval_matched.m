%% sim_eval_matched.m  (Variante A: sim() auf das Trainings-Simulink-Modell)
%
% Fuehrt den trainierten RL-Agenten auf demselben Simulink-Modell aus,
% das fuer das Training genutzt wurde, und loggt jede Episode in dasselbe
% Format wie B1 (real). Damit kann C3 die Sim/Real-Paare ueber meta.label
% direkt matchen.
%
% Voraussetzungen:
%   - Trainings-Simulink-Modell (.slx) mit RL Agent Block
%   - Trainierter Agent als .mat (RL Toolbox-Format)
%   - run_logger.m, startpose_library.m im Pfad
%
% WICHTIG: An den mit "ANPASSEN:" markierten Stellen musst du das Skript
% an deine Modell-Struktur anpassen. Drei Stellen sind betroffen:
%   1) Wie wird q0 ins Modell injiziert?
%   2) Wie heisst der RL-Agent-Block im Modell?
%   3) Welche Signale extrahierst du aus out (Namen deiner To-Workspace-Bloecke)?
%
% Wenn du das einmal eingestellt hast, laeufts deterministisch durch.

clear; clc; close all;
cd(sk_path());   % Modelle laden die Kinova-Meshes relativ zum Repository-Ordner

%% =========================
%  KONFIGURATION
%  =========================
cfg.modelName    = 'SpaceKinova_MotionProfile';      % ANPASSEN: ohne .slx
cfg.agentFile    = sk_path('SavedAgents/MotionProfile/Circle/PPO/SpaceKinova_PPO_agent_motionprofile.mat');   % ANPASSEN: Pfad zu Agent-mat
cfg.agentVarName = 'agent';                     % ANPASSEN: Variablenname im mat
cfg.agentBlockPath = 'SpaceKinova_MotionProfile/agent';                        % ANPASSEN: z.B. 'mein_modell/RL Agent'
                                                % Leer lassen wenn UseExplorationPolicy
                                                % am Agent-Objekt selbst gesetzt wird

cfg.runDir       = sk_path('data', 'hardware', 'runs');
cfg.urdfFile     = sk_path('robot', 'SpaceKinova.urdf');
cfg.eeBodyName   = 'end_effector_link';
cfg.toolOffset   = [0; 0; 0.115];
cfg.nJoints      = 7;

% Stop-Time pro Episode in Modellzeit. Auf das gleiche setzen wie deine Real-Episoden.
cfg.stopTime_s   = 8.5;

% Wenn dein Modell selbst eine StopFcn hat, die das frueher abbricht
% (z.B. bei done == true), dann diesen Wert ruhig grosszuegig lassen.

%% =========================
%  EXPERIMENT-MATRIX
%  =========================
% Spalten: label, startpose-Name, seed
% WICHTIG: Labels muessen exakt den Real-Run-Labels aus B1 entsprechen,
% sonst paart C3 die Dateien nicht.

scenarios = { ...
    'agent_zero_seed1',   'zero',         1 ; ...
    'agent_zero_seed2',   'zero',         2 ; ...
    'agent_elbow_seed1',  'elbow_bent',   1 ; ...
    'agent_elbow_seed2',  'elbow_bent',   2 ; ...
};

%% =========================
%  VORBEREITUNG
%  =========================
poses = startpose_library();
for i = 1:size(scenarios, 1)
    pName = scenarios{i, 2};
    assert(isfield(poses, pName), ...
        'Startpose "%s" nicht in startpose_library (Scenario %d).', pName, i);
end

% Agent laden
assert(isfile(cfg.agentFile), 'Agent-Datei nicht gefunden: %s', cfg.agentFile);
S = load(cfg.agentFile);
assert(isfield(S, cfg.agentVarName), ...
    'Variable "%s" nicht in %s gefunden.', cfg.agentVarName, cfg.agentFile);
agent = S.(cfg.agentVarName);

% Auf deterministische Policy umstellen
try
    agent.UseExplorationPolicy = false;
    fprintf('Agent: UseExplorationPolicy = false (deterministisch)\n');
catch
    warning(['Konnte UseExplorationPolicy nicht setzen. ' ...
             'Pruef ob dein Agent-Typ das unterstuetzt.']);
end
% Agent-Variable im Base-Workspace verfuegbar machen, weil der RL-Agent-
% Block im Modell ueblicherweise per Variablennamen referenziert.
assignin('base', cfg.agentVarName, agent);

% URDF laden (fuer Fallback-FK falls dein Modell ee_pos nicht selbst loggt)
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

% Modell laden
load_system(cfg.modelName);

%% =========================
%  HAUPTLOOP UEBER SZENARIEN
%  =========================
nScen = size(scenarios, 1);
successCount = 0;

for i = 1:nScen
    label    = scenarios{i, 1};
    poseName = scenarios{i, 2};
    seed     = scenarios{i, 3};
    q0       = poses.(poseName);

    fprintf('\n======================================\n');
    fprintf(' Sim-Szenario %d/%d: %s\n', i, nScen, label);
    fprintf('   pose=%s, seed=%d\n', poseName, seed);
    fprintf('======================================\n');

    try
        % --- Initialzustand und RNG setzen ---
        rng(seed);
        assignin('base', 'q0_deg', q0);
        assignin('base', 'q0_rad', deg2rad(q0));

        % ANPASSEN (1): Falls dein Modell eine andere Variable referenziert,
        % hier ergaenzen. Beispiele:
        %   assignin('base', 'initialJointAngles', deg2rad(q0));
        %   set_param([cfg.modelName '/Joint Init'], 'Value', mat2str(q0));
        %
        % Tipp: Wenn dein Trainings-Modell die Startpose ueber eine
        % ResetFcn / Environment-Reset zufaellig sampelt, musst du diese
        % temporaer auf q0 fixieren. Z.B. ueber einen "Eval Mode"-Schalter
        % im Modell oder durch Override der ResetFcn.

        % --- Simulation laufen lassen ---
        simIn = Simulink.SimulationInput(cfg.modelName);
        simIn = simIn.setModelParameter('StopTime', num2str(cfg.stopTime_s));
        simIn = simIn.setModelParameter('SimulationMode', 'normal');

        fprintf(' Starte sim()...\n');
        tStart = tic;
        out = sim(simIn);
        fprintf(' sim() fertig in %.2f s Wallclock\n', toc(tStart));

        % --- Logs extrahieren ---
        [t, q_meas_deg, dq_cmd_deg, ee_meas, obs_log, reward_log] = ...
            extractLogs(out, cfg, robot_rbt);

        % --- In data/meta-Schema verpacken ---
        data = struct();
        data.t           = t;
        data.q_measured  = q_meas_deg;
        data.dq_cmd      = dq_cmd_deg;
        data.ee_measured = ee_meas;
        if ~isempty(obs_log),    data.obs    = obs_log;    end
        if ~isempty(reward_log), data.reward = reward_log; end

        meta = struct();
        meta.label         = label;
        meta.source        = 'sim';
        meta.startpose     = q0;
        meta.startposeName = poseName;
        meta.seed          = seed;
        meta.modelName     = cfg.modelName;
        meta.agentFile     = cfg.agentFile;
        meta.urdfFile      = cfg.urdfFile;
        meta.stopTime_s    = cfg.stopTime_s;
        meta.comment       = sprintf('Variante A (sim() auf %s)', cfg.modelName);

        run_logger(data, meta, 'runDir', cfg.runDir, 'prefix', 'sim');
        successCount = successCount + 1;

    catch ME
        fprintf(2, ' FEHLER bei %s: %s\n', label, ME.message);
        fprintf(2, '   (in %s, Zeile %d)\n', ...
                ME.stack(1).name, ME.stack(1).line);
    end
end

fprintf('\n== %d/%d Sim-Szenarien erfolgreich gespeichert ==\n', ...
        successCount, nScen);
fprintf('Output: %s/sim_*\n', cfg.runDir);

% Modell wieder schliessen (nicht speichern!)
try
    close_system(cfg.modelName, 0);
catch
end

%% =========================
%  LOKALE FUNKTION: LOGS EXTRAHIEREN
%  =========================
function [t, q_meas_deg, dq_cmd_deg, ee_meas, obs_log, reward_log] = ...
        extractLogs(out, cfg, robot_rbt)
% Holt Signale aus der SimulationOutput. Hier sind die Annahmen ueber
% die Block-/Variable-Namen.
%
% ANPASSEN (3): Die Variablennamen unten muessen zu deinen
% To-Workspace-Bloecken im Modell passen.
%
% Erwartete Logging-Bloecke (alle als "Save format: Timeseries"):
%   q       - Gelenkwinkel-Output, [rad] oder [deg]
%   dq_cmd  - Action-Output des Agenten, [rad/s] oder [deg/s]
%   EE_pos_differenz      - (optional) EE-Position [m]
%   obs_log     - (optional) Observation-Vektor
%   reward_log  - (optional) Reward pro Step
%
% Wenn dein Modell andere Namen nutzt, hier umbiegen.

    t = []; q_meas_deg = []; dq_cmd_deg = [];
    ee_meas = []; obs_log = []; reward_log = [];

    % q -- Pflicht
    if hasField(out, 'q')
        ts = out.q;
        t  = ts.Time(:);
        q_raw = squeeze(ts.Data);
        if size(q_raw, 1) ~= numel(t) && size(q_raw, 2) == numel(t)
            q_raw = q_raw.';
        end
        % Heuristik: wenn |max| < 2*pi -> rad annehmen
        if max(abs(q_raw), [], 'all') < 2*pi + 0.1
            q_meas_deg = rad2deg(q_raw);
        else
            q_meas_deg = q_raw;   % bereits in deg
        end
    else
        error('Logging-Signal "q" nicht in SimulationOutput gefunden.');
    end

    % dq_cmd -- Pflicht
    if hasField(out, 'dq_cmd')
        ts = out.dq_cmd;
        dq_raw = squeeze(ts.Data);
        if size(dq_raw, 1) ~= numel(t) && size(dq_raw, 2) == numel(t)
            dq_raw = dq_raw.';
        end
        if max(abs(dq_raw), [], 'all') < 50
            % unter 50 -> wahrscheinlich rad/s
            dq_cmd_deg = rad2deg(dq_raw);
        else
            dq_cmd_deg = dq_raw;  % bereits deg/s
        end
    else
        error('Logging-Signal "dq_cmd" nicht in SimulationOutput gefunden.');
    end

    % ee -- optional, sonst per FK
    if hasField(out, 'EE_pos_differenz')
        ts = out.EE_pos_differenz;
        ee_raw = squeeze(ts.Data);
        if size(ee_raw, 1) ~= numel(t) && size(ee_raw, 2) == numel(t)
            ee_raw = ee_raw.';
        end
        ee_meas = ee_raw;
    else
        % Fallback: per FK aus q nachrechnen
        ee_meas = zeros(numel(t), 3);
        for k = 1:numel(t)
            T = getTransform(robot_rbt, deg2rad(q_meas_deg(k, :)), ...
                             char(cfg.eeBodyName));
            p = T(1:3, 4) + T(1:3, 1:3) * cfg.toolOffset(:);
            ee_meas(k, :) = p.';
        end
    end

    % obs -- optional
    if hasField(out, 'observation')
        ts = out.observation;
        obs_raw = squeeze(ts.Data);
        if size(obs_raw, 1) ~= numel(t) && size(obs_raw, 2) == numel(t)
            obs_raw = obs_raw.';
        end
        obs_log = obs_raw;
    end

    % reward -- optional
    if hasField(out, 'reward')
        ts = out.reward;
        r_raw = squeeze(ts.Data);
        reward_log = r_raw(:);
    end
end

function tf = hasField(out, name)
    tf = false;
    try
        tf = ~isempty(out.(name));
    catch
        tf = false;
    end
end
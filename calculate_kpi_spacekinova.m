%% calculate_kpi_spacekinova.m
% KPI-Berechnung fuer SpaceKinova (7-DOF Kinova Gen3 auf freischwebender Basis).
% Fuehrt N Episoden aus und berechnet Performance-Metriken.
%
% Voraussetzungen:
%   - SpaceKinova.urdf im Pfad
%   - Trainierter Agent als .mat-Datei (Variable 'agent')
%   - Simulink-Modell SpaceKinova_Torque.slx
%   - Funktion computeKPIsFromLogs.m im Pfad

clc; clear; close all;

%% =========================
%  1) PARAMETER / KONFIGURATION
%  =========================

% --- Anzahl Evaluations-Episoden ---
N_episodes = 50;

% --- Simulink-Modell ---
mdl = 'SpaceKinova_MotionProfile';

% --- URDF ---
urdfFile = 'SpaceKinova.urdf';
eeBodyName = 'kinova_end_effector_link';

% --- Freiheitsgrade ---
nJ = 7;

% --- Sicherheits-/Spec-Parameter ---
d_safe   = 0.02;          % Mindestabstand [m]
dt_agent = 0.025;         % Agent Rate [s] (40 Hz)

% --- Kinova Gen3 Drehmomentlimits (Nominal/Continuous) ---
% Grosse Aktuatoren J1-J4: 32 Nm, Kleine Aktuatoren J5-J7: 13 Nm
tau_max = [32; 32; 32; 32; 13; 13; 13];  % [Nm] (7x1)

% --- Gelenkpositions-Limits (aus ros_kortex URDF) ---
% J1,J3,J7: continuous -> Software-Limit 2*pi
% J2: +/-2.41 rad, J4: +/-2.66 rad, J5: +/-2.23 rad, J6: +/-2.01 rad
qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];  % [rad]
qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];  % [rad]

% --- Geschwindigkeitslimits (aus URDF) ---
% Grosse Aktuatoren J1-J4: 1.3963 rad/s, Kleine Aktuatoren J5-J7: 1.2218 rad/s
dqLim = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218]; % [rad/s]

% --- Action-Limits (70% Sicherheitsfaktor) ---
safetyFactor = 0.7;
dq_max = safetyFactor * dqLim;

% --- Simulationszeit ---
T        = 8.5;           % Episodendauer [s]
Ts       = 0.005;         % Simulations-FixedStep [s]
Ts_agent = 0.025;         % Agent SampleTime [s]

% --- Referenztrajektorie (Kreis) ---
r      = 0.2;                              % Radius [m]
center = [0.0, -0.025, 1.687 - r];        % Mittelpunkt
omega  = pi / T;                           % Winkelgeschwindigkeit
yConst = 0;                                % konstante y-Aenderung

% --- Initialwerte fuer Reward & Done ---
reward_init = 0;
isdone_init = 0;

%% =========================
%  2) PARAMETER INS BASE WORKSPACE
%  =========================
% assignin('base', 'd_safe',      d_safe);
% assignin('base', 'tau_max',     tau_max);
% assignin('base', 'dq_max',      dq_max);
% assignin('base', 'dt_agent',    dt_agent);
% assignin('base', 'nJ',          nJ);
% assignin('base', 'qLim_lower',  qLim_lower);
% assignin('base', 'qLim_upper',  qLim_upper);
% assignin('base', 'dqLim',       dqLim);
assignin('base', 'reward_init', reward_init);
assignin('base', 'isdone_init', isdone_init);

%% =========================
%  3) REFERENZTRAJEKTORIE ERZEUGEN
%  =========================
t = 0:Ts:T;

x = center(1) + r * sin(omega * t);
y = center(2) + yConst * t;
z = center(3) + r * cos(omega * t);

traj = [x(:), y(:), z(:)];

dt_traj = mean(diff(t));
vref    = [zeros(1, 3); diff(traj) / dt_traj];

EE_ref  = timeseries(traj, t);
EE_vref = timeseries(vref, t);

assignin('base', 'EE_ref',  EE_ref);
assignin('base', 'EE_vref', EE_vref);

%% =========================
%  4) ROBOTER LADEN + IK (q_des)
%  =========================
assert(isfile(urdfFile), 'URDF nicht gefunden: %s', urdfFile);

robot_rbt = importrobot(urdfFile);
robot_rbt.DataFormat = 'row';

assignin('base', 'robot_rbt',  robot_rbt);
assignin('base', 'eeBodyName', eeBodyName);

% Inverse Kinematik
ik = inverseKinematics('RigidBodyTree', robot_rbt);
ikWeights = [1 1 1 0.05 0.05 0.05];   % Position wichtig, Orientierung schwach

qSeed = homeConfiguration(robot_rbt);
q_des = zeros(numel(t), nJ);
R0    = eye(3);

for k = 1:numel(t)
    Tgoal = [R0, traj(k,:).'; 0 0 0 1];
    [qSol, ~] = ik(eeBodyName, Tgoal, ikWeights, qSeed);
    q_des(k,:) = qSol(1:nJ);
    qSeed = qSol;
end

assignin('base', 'q_des', q_des);

%% =========================
%  5) AGENT LADEN
%  =========================
% ===== HIER DEN DATEINAMEN DES TRAINIERTEN AGENTEN ANPASSEN =====
agentFile = 'SpaceKinova_PPO_agent_motionprofile.mat';
assert(isfile(agentFile), 'Agent-Datei nicht gefunden: %s', agentFile);
load(agentFile, 'agent');
fprintf('Agent geladen: %s\n', agentFile);

%% =========================
%  6) SIMULINK-MODELL KONFIGURIEREN
%  =========================
load_system(mdl);

set_param(mdl, ...
    'StopTime',   num2str(T), ...
    'Solver',     'ode14x', ...
    'FixedStep',  num2str(Ts), ...
    'SolverType', 'Fixed-step');

%% =========================
%  7) N EPISODEN SIMULIEREN
%  =========================
fprintf('Starte %d Evaluations-Episoden ...\n', N_episodes);

logsouts = cell(1, N_episodes);

for i = 1:N_episodes
    simOut = sim(mdl);
    logsouts{i} = simOut.logsout;

    if mod(i, 10) == 0
        fprintf('  Episode %d / %d abgeschlossen.\n', i, N_episodes);
    end
end

fprintf('Alle %d Episoden abgeschlossen.\n', N_episodes);

%% =========================
%  8) KPIs BERECHNEN
%  =========================
params.tau_max = tau_max(:).';              % 1x7
params.q_min   = qLim_lower(:).';          % 1x7
params.q_max   = qLim_upper(:).';          % 1x7
params.dq_max  = dqLim(:).';               % 1x7 (optional, falls computeKPIsFromLogs nutzt)
params.nJ      = nJ;
params.T_episode = T;

kpi = computeKPIsFromLogs(logsouts, params);

disp('--- KPI-Ergebnisse ---');
disp(kpi);

% %% =========================
% %  9) VISUALISIERUNG: Soll- vs. Ist-EE-Trajektorie (gemittelt)
% %  =========================
% 
% % --- Gemeinsamer Zeitvektor (aus Episode 1) ---
% ep0        = 1;
% EE_ts0     = logsouts{ep0}.getElement('p_EE').Values;
% t_common   = EE_ts0.Time(:);
% Nt         = numel(t_common);
% 
% % --- Container ---
% EE_ref_all = nan(Nt, 3, N_episodes);
% EE_ist_all = nan(Nt, 3, N_episodes);
% 
% for ep = 1:N_episodes
%     logsout = logsouts{ep};
% 
%     EE_ts   = logsout.getElement('p_EE').Values;
%     t_ep    = EE_ts.Time(:);
%     ee_ep   = reshape_time_series(EE_ts.Data);          % (Ne x 3)
% 
%     ref_ep  = interp1(EE_ref.Time, EE_ref.Data, t_ep, 'linear', 'extrap');
% 
%     EE_ref_all(:,:,ep) = interp1(t_ep, ref_ep, t_common, 'linear', 'extrap');
%     EE_ist_all(:,:,ep) = interp1(t_ep, ee_ep,  t_common, 'linear', 'extrap');
% end
% 
% EE_ref_mean = mean(EE_ref_all, 3, 'omitnan');
% EE_ist_mean = mean(EE_ist_all, 3, 'omitnan');
% EE_ist_std  = std(EE_ist_all, 0, 3, 'omitnan');
% 
% % --- Plot: XZ-Ebene (da Kreisbahn in x-z liegt) ---
% figure('Name', 'EE-Trajektorie XZ (gemittelt)');
% plot(EE_ref_mean(:,1), EE_ref_mean(:,3), 'b-', 'LineWidth', 1.5); hold on;
% plot(EE_ist_mean(:,1), EE_ist_mean(:,3), 'r--', 'LineWidth', 1.5);
% grid on; axis equal;
% xlabel('x [m]'); ylabel('z [m]');
% legend('Soll-EE-Bahn', 'Mittlere Ist-EE-Bahn', 'Location', 'best');
% title(sprintf('Endeffektortrajektorie (XZ) – Mittel ueber %d Episoden', N_episodes));

% %% =========================
% %  10) VISUALISIERUNG: Basis-Orientierung (Quaternion, gemittelt)
% %  =========================
% 
% % --- Container: (Nt x 4 x N) ---
% Q_all = nan(Nt, 4, N_episodes);
% 
% for ep = 1:N_episodes
%     logsout = logsouts{ep};
%     q_ts    = logsout.getElement('basis_ori').Values;
%     t_ep    = q_ts.Time(:);
%     q_ep    = reshape_time_series(q_ts.Data);            % (Ne x 4) [w x y z]
% 
%     Q_all(:,:,ep) = interp1(t_ep, q_ep, t_common, 'linear', 'extrap');
% end
% 
% % --- Quaternionen sign-konsistent machen (q und -q sind gleich) ---
% q_ref = squeeze(Q_all(:,:,1));
% for ep = 1:N_episodes
%     q_ep = squeeze(Q_all(:,:,ep));
%     dots = sum(q_ep .* q_ref, 2);
%     flip = dots < 0;
%     q_ep(flip,:) = -q_ep(flip,:);
%     Q_all(:,:,ep) = q_ep;
% end
% 
% Q_mean = mean(Q_all, 3, 'omitnan');
% Q_mean = Q_mean ./ vecnorm(Q_mean, 2, 2);   % normalisieren
% 
% figure('Name', 'Basis-Orientierung (gemittelt)');
% plot(t_common, Q_mean(:,1), 'LineWidth', 1.5); hold on;
% plot(t_common, Q_mean(:,2), 'LineWidth', 1.5);
% plot(t_common, Q_mean(:,3), 'LineWidth', 1.5);
% plot(t_common, Q_mean(:,4), 'LineWidth', 1.5);
% grid on;
% xlabel('Zeit [s]'); ylabel('Normiertes Quaternion');
% legend('w', 'x', 'y', 'z', 'Location', 'best');
% xlim([0, T+0.5]); ylim([-0.2, 1.2]);
% title(sprintf('Basis-Orientierung – Mittel ueber %d Episoden', N_episodes));

% % %% =========================
% % %  11) VISUALISIERUNG: Einzelepisode (optional)
% % %  =========================
% % 
% % --- Plot: Soll vs. Ist fuer Episode 1 ---
% epIdx   = 1;
% logsout = logsouts{epIdx};
% 
% EE_ts      = logsout.getElement('p_EE').Values;
% t_ep       = EE_ts.Time;
% EE_ist_ep  = reshape_time_series(EE_ts.Data);
% EE_ref_ep  = interp1(EE_ref.Time, EE_ref.Data, t_ep, 'linear', 'extrap');
% 
% figure('Name', sprintf('EE-Trajektorie Episode %d', epIdx));
% subplot(1,2,1);
% plot(EE_ref_ep(:,1), EE_ref_ep(:,3), 'b-', 'LineWidth', 1.2); hold on;
% plot(EE_ist_ep(:,1), EE_ist_ep(:,3), 'r--', 'LineWidth', 1.2);
% grid on; axis equal;
% xlabel('x [m]'); ylabel('z [m]');
% legend('Soll', 'Ist', 'Location', 'best');
% title(sprintf('XZ-Ebene – Episode %d', epIdx));
% 
% subplot(1,2,2);
% err_norm = vecnorm(EE_ref_ep - EE_ist_ep, 2, 2);
% plot(t_ep, err_norm * 1000, 'k-', 'LineWidth', 1.2);
% grid on;
% xlabel('Zeit [s]'); ylabel('Positionsfehler [mm]');
% title(sprintf('EE-Positionsfehler – Episode %d', epIdx));

%% =========================
%  HILFSFUNKTIONEN
%  =========================
function kpi = computeKPIsFromLogs(logsoutIn, params)
% computeKPIsFromLogs  Berechnet KPIs aus Simulink-Logsout-Daten.
%
% Rückgabe:
%   kpi struct mit Skalaren (gruppiert):
%
%   --- EE-Tracking ---
%     K1   EE-Position MSE [m²]
%     K2   Maximaler EE-Trackingfehler [m]
%
%   --- Basisstörung ---
%     K3   Basis-Orientierungsfehler (Mittelwert) [rad]
%     K4   Mittlere Basiswinkelgeschwindigkeit [rad/s]
%
%   --- Effizienz ---
%     K5   Mittlere Leistung [W] (zeitnormiert)
%     K6   Smoothness / Jerk
%
%   --- Robustheit ---
%     K7   Mean Episode Return
%     K8   Early-Termination-Rate (0..1)
%     K9   Mittlerer Abbruchzeitpunkt [s]
%
% params muss enthalten:
%   .T_episode   Soll-Episodendauer [s] (z.B. 8.5)
%   (optional)   .T_tolerance  Toleranz fuer Erkennung Fruehabbruch [s] (default 0.1)

    % ===== logsoutIn in Cell-Array von Dataset normalisieren =====
    if isa(logsoutIn,'Simulink.SimulationData.Dataset')
        if numel(logsoutIn) == 1
            logsoutCell = {logsoutIn};
        else
            logsoutCell = cell(numel(logsoutIn),1);
            for i = 1:numel(logsoutIn)
                logsoutCell{i} = logsoutIn(i);
            end
        end

    elseif iscell(logsoutIn)
        logsoutCell = logsoutIn;

    elseif isstruct(logsoutIn) && isfield(logsoutIn,"logsout")
        logsoutCell = cell(numel(logsoutIn),1);
        for i = 1:numel(logsoutIn)
            logsoutCell{i} = logsoutIn(i).logsout;
        end

    else
        error("computeKPIsFromLogs:UnsupportedType", ...
              "logsoutIn vom Typ %s wird nicht unterstuetzt.", class(logsoutIn));
    end

    NE = numel(logsoutCell);

    % ===== Parameter fuer Fruehabbruch-Erkennung =====
    T_episode = params.T_episode;
    if isfield(params, 'T_tolerance')
        T_tol = params.T_tolerance;
    else
        T_tol = 0.1;
    end

    % ===== Prealloc =====
    epReturn          = zeros(NE,1);
    epDuration        = zeros(NE,1);
    epEarlyStop       = false(NE,1);
    epMSE_EE          = zeros(NE,1);
    epMaxErr_EE       = zeros(NE,1);
    epJerk            = zeros(NE,1);
    epOriMean         = zeros(NE,1);
    epBaseAngVelMean  = zeros(NE,1);
    epMeanPower       = zeros(NE,1);

    for k = 1:NE
        logsout = logsoutCell{k};

        if ~isa(logsout,"Simulink.SimulationData.Dataset")
            error("Episode %d ist vom Typ %s, Dataset erwartet.", ...
                  k, class(logsout));
        end

        % ---------- Signale holen ----------
        reward   = logsout.getElement('reward').Values;
        EE_diff  = logsout.getElement('EE_pos_differenz').Values;
        tau      = logsout.getElement('tau').Values;
        ori_err  = logsout.getElement('basis_orientation_error').Values;
        w_base   = logsout.getElement('w_base').Values;
        dq       = logsout.getElement('dq').Values;

        t = reward.Time;

        % ---------- Episodendauer & Fruehabbruch ----------
        if numel(t) >= 2
            epDuration(k) = t(end) - t(1);
        else
            epDuration(k) = 0;
        end
        epEarlyStop(k) = epDuration(k) < (T_episode - T_tol);

        % ---------- Return ----------
        epReturn(k) = sum(reward.Data);

        % ---------- EE-Trackingfehler ----------
        EEdata = reshape_time_series(EE_diff.Data);
        if size(EEdata,2) == 1
            errNorm = abs(EEdata(:,1));
        else
            errNorm = vecnorm(EEdata,2,2);
        end
        epMSE_EE(k)    = mean(errNorm.^2);
        epMaxErr_EE(k) = max(errNorm);

        % ---------- Basis-Orientierungsfehler ----------
        oriData = reshape_time_series(ori_err.Data);
        if size(oriData,2) == 1
            oriNorm = abs(oriData(:,1));
        else
            oriNorm = vecnorm(oriData,2,2);
        end
        epOriMean(k) = mean(oriNorm);

        % ---------- Basiswinkelgeschwindigkeit ----------
        wBaseData = reshape_time_series(w_base.Data);
        if size(wBaseData,1) >= 1
            epBaseAngVelMean(k) = mean(vecnorm(wBaseData,2,2));
        else
            epBaseAngVelMean(k) = 0;
        end

        % ---------- Mittlere Leistung (zeitnormiert) ----------
        dqData  = reshape_time_series(dq.Data);
        tauData = reshape_time_series(tau.Data);

        Nmin = min([size(dqData,1), size(tauData,1), numel(t)]);
        dqUse  = dqData(1:Nmin,:);
        tauUse = tauData(1:Nmin,:);
        tUse   = t(1:Nmin);

        if numel(tUse) >= 2 && (tUse(end) - tUse(1)) > 0
            P = sum(abs(tauUse .* dqUse), 2);
            E = trapz(tUse, P);
            epMeanPower(k) = E / (tUse(end) - tUse(1));
        else
            epMeanPower(k) = 0;
        end

        % ---------- Smoothness / Jerk ----------
        if size(tauData,1) >= 3
            tauDD    = diff(tauData,2,1);
            jerkNorm = vecnorm(tauDD,2,2);
            epJerk(k)= mean(jerkNorm);
        else
            epJerk(k)= 0;
        end
    end

    % ======= Aggregation (gruppiert) =======

    % --- EE-Tracking ---
    kpi.K1 = mean(epMSE_EE);                     % EE-Position MSE [m²]
    kpi.K2 = max(epMaxErr_EE);                    % Max EE-Trackingfehler [m]

    % --- Basisstoerung ---
    kpi.K3 = mean(epOriMean);                     % Basis-Orientierungsfehler [rad]
    kpi.K4 = mean(epBaseAngVelMean);              % Mittlere Basiswinkelgeschw. [rad/s]

    % --- Effizienz ---
    kpi.K5 = mean(epMeanPower);                   % Mittlere Leistung [W]
    kpi.K6 = mean(epJerk);                        % Smoothness / Jerk

    % --- Robustheit ---
    kpi.K7 = mean(epReturn);                      % Mean Episode Return
    kpi.K8 = mean(epEarlyStop);                   % Early-Termination-Rate (0..1)
    if any(epEarlyStop)
        kpi.K9 = mean(epDuration(epEarlyStop));   % Mittlerer Abbruchzeitpunkt [s]
    else
        kpi.K9 = T_episode;                       % Kein Abbruch -> volle Dauer
    end

    % ======= Zusatzinfo (nicht aggregiert) =======
    kpi.epDuration  = epDuration;
    kpi.epEarlyStop = epEarlyStop;
end



% =====================================================================
function data2D = reshape_time_series(raw)
% Bringt ein timeseries.Data mit Form:
%   - chan x 1 x N
%   - 1 x chan x N
%   - 1 x 1 x N
%   - N x chan
% in die einheitliche Form:
%   N x chan

    sz = size(raw);

    if ndims(raw) == 3
        % Wir nehmen an: letzter Index = Zeit
        % typische Fälle: 3x1xN oder 4x1xN
        data2D = squeeze(permute(raw, [3 1 2]));  % N x chan
    elseif isvector(raw)
        % 1D: N oder 1xN -> N x 1
        data2D = raw(:);
    else
        % schon 2D: hoffen, dass es N x chan ist; sonst transponieren
        if sz(1) == 1 && sz(2) > 1
            data2D = raw.';                       % 1 x chan -> chan x 1
        else
            data2D = raw;
        end
    end
end
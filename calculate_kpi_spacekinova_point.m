%% calculate_kpi_spacekinova_point.m
% KPI-Berechnung fuer SpaceKinova bei Punktanfahrt-Aufgabe.
%
% Unterschied zur Trajektorien-Variante:
%   - Konstante Soll-EE-Position (kein zeitvariabler Pfad)
%   - KPIs auf Konvergenz statt Tracking ausgelegt:
%       Endfehler, Setzzeit, Min-Distanz, Pfad-Effizienz, Erfolgsrate
%   - Plots zeigen Konvergenz zum Ziel (Distanz-Zeit, 3D-Pfade)
%
% Voraussetzungen:
%   - SpaceKinova.urdf im Pfad
%   - Trainierter Agent als .mat-Datei (Variable 'agent')
%   - Simulink-Modell mit konstantem EE_ref
%   - Funktion reshape_time_series in diesem Skript

clc; clear; close all;

%% =========================
%  1) PARAMETER / KONFIGURATION
%  =========================

% --- Anzahl Evaluations-Episoden ---
N_episodes = 5;

% --- Simulink-Modell ---
mdl = 'SpaceKinova_MotionProfile_point';

% --- URDF / Robot ---
urdfFile   = 'SpaceKinova.urdf';
eeBodyName = 'kinova_end_effector_link';
nJ         = 7;

% --- Zielpunkt (URDF-Frame) ---
% target_pos = [0.0; -0.025; 1.487];   % konstante Soll-EE-Position [m]
target_pos = [0.479; -0.005; 1.136];
target_R   = eye(3);

% --- Erfolgs-/Setzzeit-Toleranz ---
tol_success  = 0.05;   % Distanz [m] fuer "Ziel erreicht"
tol_settle   = 0.05;   % Distanz [m] fuer Setzzeit-Schwelle
settle_hold  = 0.5;    % Wie lange muss man in der Toleranz bleiben [s]

% --- Sicherheits-/Spec-Parameter ---
d_safe    = 0.02;
dt_agent  = 0.1;

tau_max   = [32; 32; 32; 32; 13; 13; 13];
qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];
dq_max     = 0.7 * dqLim;

% --- Simulationszeit ---
T        = 35;
Ts       = 0.02;
Ts_agent = 0.1;

% --- Initialwerte fuer Reward & Done ---
assignin('base', 'reward_init', 0);
assignin('base', 'isdone_init', 0);

%% =========================
%  2) KONSTANTE EE-REFERENZ
%  =========================
t = (0:Ts:T)';
N = numel(t);

traj = repmat(target_pos.', N, 1);   % konstantes Ziel
vref = zeros(N, 3);                   % keine Soll-Geschwindigkeit

EE_ref  = timeseries(traj, t);
EE_vref = timeseries(vref, t);

assignin('base', 'EE_ref',  EE_ref);
assignin('base', 'EE_vref', EE_vref);

%% =========================
%  3) ROBOTER LADEN + IK FUER ANKER-POSE
%  =========================
assert(isfile(urdfFile), 'URDF nicht gefunden: %s', urdfFile);

robot_rbt = importrobot(urdfFile);
robot_rbt.DataFormat = 'row';
assignin('base', 'robot_rbt',  robot_rbt);
assignin('base', 'eeBodyName', eeBodyName);

ik        = inverseKinematics('RigidBodyTree', robot_rbt);
ikWeights = [0.05 0.05 0.05  1 1 1];
qSeed     = homeConfiguration(robot_rbt);
Tgoal     = [target_R, target_pos; 0 0 0 1];
[qSol, ~] = ik(eeBodyName, Tgoal, ikWeights, qSeed);
q_target  = qSol(1:nJ).';
assignin('base', 'q_target', q_target);
assignin('base', 'q0',       q_target);
assignin('base', 'dq0',      zeros(nJ,1));

% Fuer Kompatibilitaet mit Modellen, die q_des erwarten
q_des = repmat(q_target.', N, 1);
assignin('base', 'q_des', q_des);

%% =========================
%  4) AGENT LADEN
%  =========================
agentFile = 'SavedAgents/MotionProfile/point/test_agent_rand1.mat';
assert(isfile(agentFile), 'Agent-Datei nicht gefunden: %s', agentFile);
load(agentFile, 'agent');
fprintf('Agent geladen: %s\n', agentFile);

%% =========================
%  5) SIMULINK-MODELL KONFIGURIEREN
%  =========================
load_system(mdl);
set_param(mdl, ...
    'StopTime',   num2str(T), ...
    'Solver',     'ode14x', ...
    'FixedStep',  num2str(Ts), ...
    'SolverType', 'Fixed-step');

%% =========================
%  6) N EPISODEN SIMULIEREN
%  =========================
fprintf('Starte %d Evaluations-Episoden ...\n', N_episodes);
logsouts = cell(1, N_episodes);

for i = 1:N_episodes
    simOut = sim(mdl);
    logsouts{i} = simOut.logsout;
    fprintf('  Episode %d / %d abgeschlossen.\n', i, N_episodes);
end

%% =========================
%  7) KPIs BERECHNEN
%  =========================
params = struct();
params.target_pos  = target_pos(:).';
params.tol_success = tol_success;
params.tol_settle  = tol_settle;
params.settle_hold = settle_hold;
params.T_episode   = T;
params.tau_max     = tau_max(:).';
params.nJ          = nJ;

kpi = computeKPIsPointReaching(logsouts, params);

%% =========================
%  8) KPI AUSGEBEN
%  =========================
fprintf('\n========= KPIs: Punktanfahrt =========\n');
fprintf('  Anzahl Episoden       : %d\n', N_episodes);
fprintf('  Toleranz Erfolg       : %.0f mm\n', tol_success*1000);
fprintf('\n  --- Konvergenz ---\n');
fprintf('  K1 Endfehler          : %.1f mm (mean) / %.1f mm (max)\n', ...
        kpi.K1_finalErr_mean*1000, kpi.K1_finalErr_max*1000);
fprintf('  K2 Min-Distanz        : %.1f mm (mean)\n', kpi.K2_minErr_mean*1000);
fprintf('  K3 Setzzeit           : %.2f s (mean, %.0f%% der erfolgreichen Episoden)\n', ...
        kpi.K3_settle_mean, 100*kpi.K3_settle_rate);
fprintf('  K4 Erfolgsrate        : %.0f %%\n', 100*kpi.K4_success_rate);
fprintf('\n  --- Pfad-Qualitaet ---\n');
fprintf('  K5 Pfadlaenge         : %.3f m (mean)\n', kpi.K5_pathLen_mean);
fprintf('  K6 Pfad-Effizienz     : %.2f (mean, 1.0 = optimal)\n', kpi.K6_pathEff_mean);
fprintf('  K7 Max-Ueberschwingen : %.1f mm (mean)\n', kpi.K7_overshoot_mean*1000);
fprintf('\n  --- Basis-Stoerung ---\n');
fprintf('  K8 Orient.-Fehler     : %.3f rad (mean)\n', kpi.K8_oriErr_mean);
fprintf('  K9 Basis-w (Norm)     : %.3f rad/s (mean)\n', kpi.K9_baseW_mean);
fprintf('\n  --- Effizienz ---\n');
fprintf('  K10 Mittlere Leistung : %.2f W\n', kpi.K10_power_mean);
fprintf('  K11 Smoothness (Jerk) : %.2f\n', kpi.K11_jerk_mean);
fprintf('\n  --- Reward ---\n');
fprintf('  K12 Mean Return       : %.1f\n', kpi.K12_return_mean);
fprintf('=======================================\n\n');

%% =========================
%  9) VISUALISIERUNG: Distanz zum Ziel ueber Zeit
%  =========================
% Gemeinsame Zeitachse aus Episode 1
EE_ts0   = logsouts{1}.getElement('p_EE').Values;
t_common = EE_ts0.Time(:);
Nt       = numel(t_common);

dist_all = nan(Nt, N_episodes);
ee_all   = nan(Nt, 3, N_episodes);

for ep = 1:N_episodes
    EE_ts = logsouts{ep}.getElement('p_EE').Values;
    t_ep  = EE_ts.Time(:);
    ee_ep = reshape_time_series(EE_ts.Data);

    ee_interp = interp1(t_ep, ee_ep, t_common, 'linear', 'extrap');
    ee_all(:,:,ep) = ee_interp;

    dist_all(:,ep) = vecnorm(ee_interp - target_pos.', 2, 2);
end

dist_mean = mean(dist_all, 2, 'omitnan');
dist_std  = std(dist_all, 0, 2, 'omitnan');
dist_min  = min(dist_all, [], 2);
dist_max  = max(dist_all, [], 2);

figure('Name', 'Distanz zum Ziel ueber Zeit', 'Position', [100 100 800 500]);
hold on; grid on;

% Hellgraue Einzelepisoden
for ep = 1:N_episodes
    plot(t_common, dist_all(:,ep)*1000, '-', 'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
end

% Min-Max Schattierung
fill([t_common; flipud(t_common)], ...
     [dist_min*1000; flipud(dist_max*1000)], ...
     [0.8 0.85 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.4);

% Mean
plot(t_common, dist_mean*1000, 'b-', 'LineWidth', 2);

% Erfolgs-Toleranzlinie
yline(tol_success*1000, 'g--', sprintf('Toleranz %.0f mm', tol_success*1000), ...
      'LineWidth', 1.5);

xlabel('Zeit [s]');
ylabel('Distanz zum Ziel [mm]');
title(sprintf('Konvergenz zum Zielpunkt (%d Episoden)', N_episodes));
legend({'Einzelepisoden', 'Min-Max-Hülle', 'Mittelwert', 'Erfolgs-Toleranz'}, ...
       'Location', 'best');
xlim([0 T+0.2]);

%% =========================
% 10) VISUALISIERUNG: EE-Pfade im 3D mit Zielpunkt
%  =========================
figure('Name', 'EE-Pfade 3D', 'Position', [100 100 800 600]);
hold on; grid on;

cmap = lines(N_episodes);
for ep = 1:N_episodes
    ee = squeeze(ee_all(:,:,ep));
    plot3(ee(:,1), ee(:,2), ee(:,3), '-', 'Color', cmap(ep,:), 'LineWidth', 1);
    plot3(ee(1,1), ee(1,2), ee(1,3), 'o', 'Color', cmap(ep,:), ...
          'MarkerSize', 8, 'LineWidth', 1.5);  % Start
end

% Ziel
plot3(target_pos(1), target_pos(2), target_pos(3), 'kp', ...
      'MarkerSize', 20, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);

% Toleranz-Kugel um Ziel
[xs, ys, zs] = sphere(20);
surf(target_pos(1) + tol_success*xs, ...
     target_pos(2) + tol_success*ys, ...
     target_pos(3) + tol_success*zs, ...
     'FaceColor', 'g', 'FaceAlpha', 0.15, 'EdgeColor', 'none');

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Endeffektor-Pfade zum Zielpunkt');
view(45, 25); axis equal;
legend_entries = arrayfun(@(i) sprintf('Episode %d', i), 1:N_episodes, 'uni', 0);
legend([legend_entries, {'Start (Kreise)', 'Ziel', 'Toleranz-Kugel'}], ...
       'Location', 'eastoutside');

%% =========================
% 11) VISUALISIERUNG: EE-Komponenten ueber Zeit
%  =========================
ee_mean = mean(ee_all, 3, 'omitnan');
ee_std  = std(ee_all, 0, 3, 'omitnan');

figure('Name', 'EE-Position Komponenten', 'Position', [100 100 900 600]);
labels = {'x [m]', 'y [m]', 'z [m]'};
for axIdx = 1:3
    subplot(3,1,axIdx); hold on; grid on;

    % Einzelepisoden hellgrau
    for ep = 1:N_episodes
        plot(t_common, squeeze(ee_all(:,axIdx,ep)), '-', ...
             'Color', [0.85 0.85 0.85]);
    end

    % Mean
    plot(t_common, ee_mean(:,axIdx), 'b-', 'LineWidth', 2);

    % Ziel-Linie
    yline(target_pos(axIdx), 'r--', 'LineWidth', 1.5);

    xlabel('Zeit [s]'); ylabel(labels{axIdx});
    xlim([0 T+0.2]);
    if axIdx == 1
        title(sprintf('EE-Position vs. Ziel-Komponenten (%d Episoden)', N_episodes));
        legend({'Episoden', 'Mittelwert', 'Ziel'}, 'Location', 'best');
    end
end

%% =========================
% 12) VISUALISIERUNG: Basis-Orientierung (gemittelt)
%  =========================
Q_all = nan(Nt, 4, N_episodes);
for ep = 1:N_episodes
    q_ts = logsouts{ep}.getElement('basis_ori').Values;
    t_ep = q_ts.Time(:);
    q_ep = reshape_time_series(q_ts.Data);
    Q_all(:,:,ep) = interp1(t_ep, q_ep, t_common, 'linear', 'extrap');
end

% Sign-konsistent machen
q_ref = squeeze(Q_all(:,:,1));
for ep = 1:N_episodes
    q_ep = squeeze(Q_all(:,:,ep));
    flip = sum(q_ep .* q_ref, 2) < 0;
    q_ep(flip,:) = -q_ep(flip,:);
    Q_all(:,:,ep) = q_ep;
end

Q_mean = mean(Q_all, 3, 'omitnan');
Q_mean = Q_mean ./ vecnorm(Q_mean, 2, 2);

figure('Name', 'Basis-Orientierung', 'Position', [100 100 800 400]);
plot(t_common, Q_mean, 'LineWidth', 1.5);
grid on;
xlabel('Zeit [s]'); ylabel('Normiertes Quaternion');
legend('w', 'x', 'y', 'z', 'Location', 'best');
xlim([0 T+0.2]); ylim([-0.3 1.2]);
title(sprintf('Basis-Orientierung (Mittel ueber %d Episoden)', N_episodes));

%% =========================
% 13) VISUALISIERUNG: Einzelepisode Detail
%  =========================
epIdx = 1;
EE_ts = logsouts{epIdx}.getElement('p_EE').Values;
t_ep  = EE_ts.Time;
ee_ep = reshape_time_series(EE_ts.Data);
dist_ep = vecnorm(ee_ep - target_pos.', 2, 2);

figure('Name', sprintf('Episode %d Detail', epIdx), 'Position', [100 100 1100 400]);

subplot(1,3,1);
plot3(ee_ep(:,1), ee_ep(:,2), ee_ep(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(ee_ep(1,1), ee_ep(1,2), ee_ep(1,3), 'go', 'MarkerSize', 10, ...
      'MarkerFaceColor', 'g');
plot3(target_pos(1), target_pos(2), target_pos(3), 'rp', 'MarkerSize', 15, ...
      'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('3D-Pfad');
legend({'Pfad', 'Start', 'Ziel'}, 'Location', 'best');
view(45, 25);

subplot(1,3,2);
plot(t_ep, dist_ep*1000, 'b-', 'LineWidth', 1.5); hold on;
yline(tol_success*1000, 'g--', 'LineWidth', 1.5);
grid on;
xlabel('Zeit [s]'); ylabel('Distanz zum Ziel [mm]');
title('Konvergenz');
legend({'Distanz', 'Toleranz'}, 'Location', 'best');

subplot(1,3,3);
labels_xyz = {'x', 'y', 'z'};
for axIdx = 1:3
    plot(t_ep, ee_ep(:,axIdx), 'LineWidth', 1.2); hold on;
    yline(target_pos(axIdx), '--');
end
grid on;
xlabel('Zeit [s]'); ylabel('Position [m]');
title('Komponenten');
legend(labels_xyz, 'Location', 'best');

sgtitle(sprintf('Episode %d', epIdx));

%% =========================
% 14) dq_cmd speichern (Kompatibilitaet mit altem Workflow)
%  =========================
try
    dq_ts = logsouts{1}.getElement('dq_cmd').Values;
    t_dq  = dq_ts.Time;
    dq    = reshape_time_series(dq_ts.Data);
    save('dq_cmd_point.mat', 't_dq', 'dq', 'target_pos');
    fprintf('dq_cmd_point.mat gespeichert.\n');
catch
    fprintf('(dq_cmd nicht im logsout - skip.)\n');
end


%% =========================================================================
%  HILFSFUNKTIONEN
%  =========================================================================

function kpi = computeKPIsPointReaching(logsoutIn, params)
% KPIs fuer Punktanfahrt:
%   K1  Endfehler [m]
%   K2  Minimaler Fehler [m]
%   K3  Setzzeit (in Toleranz und bleibend) [s]
%   K4  Erfolgsrate
%   K5  Pfadlaenge [m]
%   K6  Pfad-Effizienz (direkt/tatsaechlich) [-]
%   K7  Ueberschwingen (max Distanz nach erstem Erreichen) [m]
%   K8  Basis-Orientierungsfehler [rad]
%   K9  Basis-Winkelgeschwindigkeit [rad/s]
%   K10 Mittlere Leistung [W]
%   K11 Smoothness (Jerk)
%   K12 Episode Return

    if isa(logsoutIn,'Simulink.SimulationData.Dataset')
        logsoutCell = {logsoutIn};
    elseif iscell(logsoutIn)
        logsoutCell = logsoutIn;
    else
        error('Unsupported logsout type');
    end

    NE         = numel(logsoutCell);
    target     = params.target_pos(:).';
    tol_ok     = params.tol_settle;
    hold_time  = params.settle_hold;
    tol_succ   = params.tol_success;

    epFinalErr   = zeros(NE,1);
    epMinErr     = zeros(NE,1);
    epSettle     = nan(NE,1);
    epSuccess    = false(NE,1);
    epPathLen    = zeros(NE,1);
    epPathEff    = zeros(NE,1);
    epOvershoot  = zeros(NE,1);
    epOriMean    = zeros(NE,1);
    epBaseW      = zeros(NE,1);
    epPower      = zeros(NE,1);
    epJerk       = zeros(NE,1);
    epReturn     = zeros(NE,1);

    for k = 1:NE
        ls = logsoutCell{k};

        EE_ts   = ls.getElement('p_EE').Values;
        reward  = ls.getElement('reward').Values;
        tau     = ls.getElement('tau').Values;
        ori_err = ls.getElement('basis_orientation_error').Values;
        w_base  = ls.getElement('w_base').Values;
        dq      = ls.getElement('dq').Values;

        t  = EE_ts.Time(:);
        ee = reshape_time_series(EE_ts.Data);

        % Distanz zum Ziel ueber Zeit
        dist = vecnorm(ee - target, 2, 2);

        % --- Konvergenz ---
        epFinalErr(k) = dist(end);
        epMinErr(k)   = min(dist);
        epSuccess(k)  = dist(end) < tol_succ;

        % --- Setzzeit (erstes Eintreten in tol_ok, das mindestens hold_time bleibt) ---
        in_tol = dist < tol_ok;
        if any(in_tol)
            idx_in = find(in_tol);
            for ii = 1:numel(idx_in)
                t_start = t(idx_in(ii));
                t_end_check = t_start + hold_time;
                mask = t >= t_start & t <= min(t_end_check, t(end));
                if all(dist(mask) < tol_ok) && (t(end) - t_start) >= hold_time
                    epSettle(k) = t_start;
                    break;
                end
            end
        end

        % --- Pfadlaenge ---
        if size(ee,1) >= 2
            seg = vecnorm(diff(ee), 2, 2);
            epPathLen(k) = sum(seg);
            direct       = norm(ee(end,:) - ee(1,:));
            % Effizienz: direkter Weg zum Ziel / tatsaechlich gegangen
            % (Werte > 1 unmoeglich, ausser numerisch)
            direct_to_target = norm(target - ee(1,:));
            if epPathLen(k) > 1e-6
                epPathEff(k) = min(1, direct_to_target / epPathLen(k));
            end
        end

        % --- Ueberschwingen: max Distanz nach Erstkontakt mit Toleranz ---
        if any(in_tol)
            idx_first = find(in_tol, 1, 'first');
            if idx_first < numel(dist)
                epOvershoot(k) = max(dist(idx_first:end)) - tol_ok;
                epOvershoot(k) = max(0, epOvershoot(k));
            end
        end

        % --- Basis-Orientierung & Winkelgeschw. ---
        oriData = reshape_time_series(ori_err.Data);
        if size(oriData,2) == 1
            oriNorm = abs(oriData);
        else
            oriNorm = vecnorm(oriData, 2, 2);
        end
        epOriMean(k) = mean(oriNorm);

        wData = reshape_time_series(w_base.Data);
        epBaseW(k) = mean(vecnorm(wData, 2, 2));

        % --- Leistung (zeitnormiert) ---
        dqData  = reshape_time_series(dq.Data);
        tauData = reshape_time_series(tau.Data);
        Nmin    = min([size(dqData,1), size(tauData,1), numel(t)]);
        if Nmin >= 2
            P = sum(abs(tauData(1:Nmin,:) .* dqData(1:Nmin,:)), 2);
            E = trapz(t(1:Nmin), P);
            epPower(k) = E / (t(Nmin) - t(1));
        end

        % --- Jerk ---
        if size(tauData,1) >= 3
            tauDD     = diff(tauData, 2, 1);
            epJerk(k) = mean(vecnorm(tauDD, 2, 2));
        end

        % --- Return ---
        epReturn(k) = sum(reward.Data);
    end

    % Aggregation
    kpi.K1_finalErr_mean  = mean(epFinalErr);
    kpi.K1_finalErr_max   = max(epFinalErr);
    kpi.K2_minErr_mean    = mean(epMinErr);
    kpi.K3_settle_mean    = mean(epSettle, 'omitnan');
    kpi.K3_settle_rate    = mean(~isnan(epSettle));
    kpi.K4_success_rate   = mean(epSuccess);
    kpi.K5_pathLen_mean   = mean(epPathLen);
    kpi.K6_pathEff_mean   = mean(epPathEff);
    kpi.K7_overshoot_mean = mean(epOvershoot);
    kpi.K8_oriErr_mean    = mean(epOriMean);
    kpi.K9_baseW_mean     = mean(epBaseW);
    kpi.K10_power_mean    = mean(epPower);
    kpi.K11_jerk_mean     = mean(epJerk);
    kpi.K12_return_mean   = mean(epReturn);

    % Roh-Werte pro Episode (optional zur Inspektion)
    kpi.raw.finalErr  = epFinalErr;
    kpi.raw.minErr    = epMinErr;
    kpi.raw.settle    = epSettle;
    kpi.raw.success   = epSuccess;
    kpi.raw.pathLen   = epPathLen;
    kpi.raw.pathEff   = epPathEff;
    kpi.raw.overshoot = epOvershoot;
    kpi.raw.return    = epReturn;
end


function data2D = reshape_time_series(raw)
    sz = size(raw);
    if ndims(raw) == 3
        data2D = squeeze(permute(raw, [3 1 2]));
    elseif isvector(raw)
        data2D = raw(:);
    else
        if sz(1) == 1 && sz(2) > 1
            data2D = raw.';
        else
            data2D = raw;
        end
    end
end
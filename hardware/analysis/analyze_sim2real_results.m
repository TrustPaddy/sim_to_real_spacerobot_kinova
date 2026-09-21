% file: analyze_sim2real_results.m
% Vergleicht die wichtigsten Real-Deploy-Läufe
% Setzt voraus dass du die richtigen Files manuell angibst

clear; clc; close all;

% --- Konfiguration: welche Files vergleichen ---
runs = {
    struct('file', sk_path('data', 'hardware', 'deploy_logs', 'run_009_agent_train_seed2_singular.mat'), ...
           'label', '40-Hz-Agent', 'color', 'r'); ...
    struct('file', sk_path('data', 'hardware', 'deploy_logs', 'run_012_agent_train_seed10_okay.mat'), ...
           'label', '10-Hz-Agent', 'color', 'b'); ...
};

% --- Lade alle ---
data = cell(length(runs), 1);
for i = 1:length(runs)
    L = load(runs{i}.file);
    data{i} = L;
end

% --- Tabelle der Kennzahlen ---
fprintf('\n%-35s %-10s %-12s %-10s %-12s\n', ...
        'Lauf', 'Steps', 'OOD-Zeit', 'RMS ep', 'Max ep');
fprintf('%s\n', repmat('-', 1, 80));
for i = 1:length(runs)
    L = data{i};
    ep_norms = L.data.ep_norm;
    if max(ep_norms) > 0.4
        ood_idx = find(ep_norms > 0.4, 1);
        ood_time = L.data.t_wall(ood_idx);
        ood_str = sprintf('%.2f s', ood_time);
    else
        ood_str = 'kein OOD';
    end
    fprintf('%-35s %-10d %-12s %-10.4f %-12.4f\n', ...
            runs{i}.label, length(L.data.t), ood_str, ...
            sqrt(mean(ep_norms.^2)), max(ep_norms));
end

% --- Plot 1: xz-Bahn-Vergleich ---
figure('Name', 'xz-Vergleich', 'Position', [100 100 800 600]);
hold on;
% Geplante Soll-Bahn
t_full = linspace(0, 8.5, 200);
% (Soll-Trajektorie aus deiner referenceTrajectory-Funktion oder hardcoded)
r = 0.2; cx = 0.0; cz = 1.487;  % an deine Werte anpassen
soll_x = cx + r * sin(pi/8.5 * t_full);
soll_z = cz + r * cos(pi/8.5 * t_full);
plot(soll_x, soll_z, 'k--', 'LineWidth', 1.5);
% Ist-Bahnen
for i = 1:length(runs)
    L = data{i};
    plot(L.data.ee_measured(:,1), L.data.ee_measured(:,3), ...
         'Color', runs{i}.color, 'LineWidth', 1.3);
end
grid on; axis equal;
xlabel('x [m]'); ylabel('z [m]');
title('End-Effector Trajektorie: Soll vs. Real');
legend(['Soll (geplanter Halbkreis)', cellfun(@(r) r.label, runs, 'UniformOutput', false)']);

% % --- Plot 2: Tracking-Fehler über Zeit ---
% figure('Name', 'Tracking-Fehler', 'Position', [100 100 800 400]);
% hold on;
% for i = 1:length(runs)
%     L = data{i};
%     plot(L.data.t_wall, L.data.ep_norm, ...
%          'Color', runs{i}.color, 'LineWidth', 1.3);
% end
% yline(0.4, 'k--', 'OOD-Schwelle');
% grid on;
% xlabel('Wallclock-Zeit [s]'); ylabel('||ep|| [m]');
% title('Positionsfehler über Zeit');
% legend(cellfun(@(r) r.label, runs, 'UniformOutput', false));

% --- Plot 3: Loop-Timing pro Lauf ---
figure('Name', 'Timing-Diagnose', 'Position', [100 100 800 400]);
for i = 1:length(runs)
    L = data{i};
    if isfield(L.data, 't_wall')
        dt = diff(L.data.t_wall);
        subplot(1, length(runs), i);
        histogram(dt*1000, 30);
        xlabel('dt [ms]'); ylabel('Anzahl');
        title(sprintf('%s\nMedian %.1f ms', runs{i}.label, median(dt)*1000));
        grid on;
    end
end

% --- Plot 4: Action-Verlauf pro aktiven Joints ---
figure('Name', 'Agent-Actions', 'Position', [100 100 800 600]);
joints_active = [2, 4, 6];
for ji = 1:length(joints_active)
    j = joints_active(ji);
    subplot(length(joints_active), 1, ji);
    hold on;
    for i = 1:length(runs)
        L = data{i};
        plot(L.data.t_wall, L.data.dq_cmd(:, j), ...
             'Color', runs{i}.color, 'LineWidth', 1.0);
    end
    grid on;
    ylabel(sprintf('dq_{cmd} J%d [deg/s]', j));
    if ji == 1
        title('Vom Agent kommandierte Geschwindigkeiten');
        legend(cellfun(@(r) r.label, runs, 'UniformOutput', false));
    end
end
xlabel('Wallclock-Zeit [s]');
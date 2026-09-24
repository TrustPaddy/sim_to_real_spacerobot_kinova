function T = desktop_d8_joint_thirds()
%DESKTOP_D8_JOINT_THIRDS  Gelenkbefehle je Bahndrittel fuer Sec. IV-D (Desktop-Plan D8, Befund A51).
%   T = desktop_d8_joint_thirds()
%
%   Deterministische Episoden (40 Hz, 65 kg, Halbkreis 8,5 s) fuer Optimized, CDR2-4 und das fruehere PPO
%   (SpaceKinova_PPO_agent_motionprofile.mat). Je Drittel der Bahn: mittlere Rohaktion und mittlerer Befehl nach der
%   Signalkette (a_rl, rad/s) fuer J2, J4 und J6, dazu der Bereich der Gelenkwinkel.
%   Ergebnis: data/simulation/desktop/D8_joint_thirds.csv

setup_project;
desktop_build_model();
ag = {'Optimized', sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'Optimized.mat');
      'CDR2-4', sk_path('SavedAgents', 'MotionProfile', 'CDR', 'PPO', 'CDR2-4.mat');
      'PPO_base', sk_path('SavedAgents', 'MotionProfile', 'Circle', 'PPO', 'SpaceKinova_PPO_agent_motionprofile.mat')};
edges = [0 8.5/3 2*8.5/3 8.5 + 1e-9];
rows = {};
for a = 1:size(ag, 1)
    r = desktop_run_episode(desktop_config('agentFile', ag{a, 2}, 'agentLabel', ag{a, 1}));
    t = r.ts.t_a_rl; tq = r.ts.t_q_cmd;
    for w = 1:3
        k = t >= edges(w) & t < edges(w + 1);
        kq = tq >= edges(w) & tq < edges(w + 1);
        row = struct('agent', ag{a, 1}, 'third', w, 't_from', edges(w), 't_to', min(edges(w + 1), 8.5));
        for j = [2 4 6]
            row.(sprintf('a_raw_J%d', j)) = mean(r.ts.a_raw(k, j));
            row.(sprintf('cmd_J%d_radps', j)) = mean(r.ts.a_rl(k, j));
            row.(sprintf('q_J%d_min_deg', j)) = rad2deg(min(r.ts.q_cmd(kq, j)));
            row.(sprintf('q_J%d_max_deg', j)) = rad2deg(max(r.ts.q_cmd(kq, j)));
        end
        rows{end + 1} = row; %#ok<AGROW>
    end
end
T = struct2table([rows{:}]);
disp(T(:, {'agent', 'third', 'cmd_J2_radps', 'cmd_J4_radps', 'cmd_J6_radps', 'q_J2_min_deg', 'q_J4_max_deg'}));
writetable(T, sk_path('data', 'simulation', 'desktop', 'D8_joint_thirds.csv'));
end

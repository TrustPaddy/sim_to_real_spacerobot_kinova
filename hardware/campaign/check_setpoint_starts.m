function T = check_setpoint_starts(varargin)
%CHECK_SETPOINT_STARTS  Anfahrtest und Freigabe der Set-Point-Posen im Labor.
%
%   CHECK_SETPOINT_STARTS() faehrt nacheinander die Zielpose Z00 und alle Startposen
%   aus hardware/campaign/setpoint_starts.mat an. Jede Pose wird vom Anker aus mit
%   ReachJointAngles angefahren. Danach fragt das Skript, ob die Pose frei von
%   Hindernissen ist (Satelliten-Mockup, zweiter Roboter, Linearachse, Kabel), und
%   faehrt zurueck zum Anker.
%   CHECK_SETPOINT_STARTS('ids', {'S05', 'S06'}) prueft nur die genannten Posen.
%   CHECK_SETPOINT_STARTS('dryRun', true) zeigt nur den Ablauf.
%
%   Jede Antwort wird mit der gemessenen Kortex-tool_pose und der aus der FK
%   vorhergesagten Position an data/hardware/campaign/setpoint_start_check.csv
%   angehaengt. Die Abweichung prueft nebenbei den Werkzeug-Offset (kortex_frames).
%   deploy_setpoint_v24 faehrt nur Posen, die fuer die aktuelle Startliste (gleicher
%   MD5) freigegeben sind. Wird die Liste neu erzeugt, muessen die Posen neu
%   freigegeben werden.
%
%   Eingaben pro Pose: ENTER faehrt hin, "s" ueberspringt, "q" beendet. Nach der
%   Anfahrt: "j" gibt frei, "n" sperrt (mit kurzer Begruendung).

p = inputParser;
p.addParameter('ids', {});
p.addParameter('dryRun', false);
p.addParameter('operatorNote', '');
p.parse(varargin{:});
o = p.Results;

cfg = struct();
cfg.nJ = 7;
cfg.kinovaIP = '192.168.0.10'; cfg.kinovaUser = 'admin'; cfg.kinovaPassword = 'admin';
cfg.sessionTimeoutMs = uint32(60000); cfg.controlTimeoutMs = uint32(200); cfg.speedCmdDuration = 0;
cfg.homingTimeout_s = 60; cfg.homingTol_deg = 1.0; cfg.homingSettle_s = 1.0;
cfg.qAnchor = deg2rad([0; 15; 180; -130; 0; 55; 90]);
cfg.startsFile = sk_path('hardware', 'campaign', 'setpoint_starts.mat');
if o.dryRun
    outFile = sk_path('data', 'hardware', 'campaign', '_dryrun', 'setpoint_start_check_dryrun.csv');
else
    outFile = sk_path('data', 'hardware', 'campaign', 'setpoint_start_check.csv');
end

L = load(cfg.startsFile);
S = L.S;
listMd5 = file_md5(cfg.startsFile);
rbt = importrobot(sk_path('robot', 'SpaceKinova.urdf'));
rbt.DataFormat = 'row';
F = kortex_frames(rbt);
env = campaign_env_meta();

poses = struct('id', 'Z00', 'q', deg2rad(S.target.q_check_deg(:)), 'd0', 0);
for i = 1:numel(S.starts)
    poses(end+1) = struct('id', S.starts(i).id, 'q', deg2rad(S.starts(i).q_deg(:)), ...
                          'd0', S.starts(i).d0_m); %#ok<AGROW>
end
if ~isempty(o.ids)
    poses = poses(ismember({poses.id}, o.ids));
end
fprintf('\n=== Anfahrtest %d Posen, Startliste MD5 %s ===\n', numel(poses), listMd5);
fprintf('E-Stop in der Hand. Jede Anfahrt startet am Anker [0 15 180 -130 0 55 90] deg.\n');

apiHandle = [];
if ~o.dryRun
    apiHandle = kinova_open(cfg);
    cleanupObj = onCleanup(@() kinova_safe_shutdown(apiHandle, cfg.nJ));
    input('ENTER faehrt zum Anker... ', 's');
    kinova_move_joints(apiHandle, cfg.qAnchor, cfg);
end

rows = {};
for i = 1:numel(poses)
    ps = poses(i);
    fprintf('\n-- %s (d0 = %.2f m), q = [%s] deg\n', ps.id, ps.d0, num2str(round(rad2deg(ps.q(:).'), 1)));
    if o.dryRun
        fprintf('   [Trockenlauf] Anker -> %s -> Anker\n', ps.id);
        continue;
    end
    a = lower(strtrim(input('   ENTER faehrt hin, s = ueberspringen, q = beenden: ', 's')));
    if strcmp(a, 'q'), break; end
    if strcmp(a, 's'), continue; end
    kinova_move_joints(apiHandle, ps.q, cfg);
    st = kinova_read_state(apiHandle, cfg);
    qm = st.q_rad(:).';
    Te = getTransform(rbt, qm, 'kinova_end_effector_link');
    Tinv = F.T_urdf_from_real \ eye(4);
    tool = Te(1:3, 4) + Te(1:3, 1:3) * F.toolOffset;
    pred = (Tinv(1:3, 1:3) * tool + Tinv(1:3, 4)).';
    meas = st.pose_real(1:3);
    fprintf('   Kortex tool_pose [%s] m, aus FK vorhergesagt [%s] m, Abweichung %.1f mm\n', ...
            num2str(meas, '%.3f '), num2str(pred, '%.3f '), 1e3 * norm(meas - pred));
    v = '';
    while ~any(strcmp(v, {'j', 'n'}))
        v = lower(strtrim(input('   Pose frei und sicher? j/n: ', 's')));
    end
    note = o.operatorNote;
    if strcmp(v, 'n')
        note = strtrim([note ' ' input('   Grund: ', 's')]);
    end
    rows(end+1, :) = {env.datetimeStart, char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ps.id, ...
                      double(strcmp(v, 'j')), listMd5, rad2deg(unwrap_to(qm, ps.q.')), meas, pred, ...
                      norm(meas - pred), note, env.gitHash}; %#ok<AGROW>
    append_row(outFile, rows(end, :));
    kinova_move_joints(apiHandle, cfg.qAnchor, cfg);
end
T = rows;
fprintf('\nErgebnisse: %s\n', outFile);
end

function q = unwrap_to(q, ref)
q = q + 2*pi * round((ref - q) / (2*pi));
end

function append_row(path, r)
newFile = ~isfile(path);
d = fileparts(path);
if ~isfolder(d), mkdir(d); end
fid = fopen(path, 'a');
c = onCleanup(@() fclose(fid));
if newFile
    fprintf(fid, ['session,datetime,id,approved,listMd5,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,q6_deg,q7_deg,' ...
                  'tool_meas_x,tool_meas_y,tool_meas_z,tool_pred_x,tool_pred_y,tool_pred_z,' ...
                  'tool_residual_m,note,gitHash\n']);
end
fprintf(fid, '%s,%s,%s,%d,%s,%s,%s,%s,%.4f,"%s",%s\n', r{1}, r{2}, r{3}, r{4}, r{5}, ...
        strjoin(compose('%.2f', r{6}), ','), strjoin(compose('%.4f', r{7}), ','), ...
        strjoin(compose('%.4f', r{8}), ','), r{9}, strrep(r{10}, '"', ''''), r{11});
end

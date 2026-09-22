%% legacy_fk_numbers.m
% Hardware-Zahlen des Papers aus den vorhandenen Logs, fuer die die Vorwaertskinematik noetig ist.
%   1) Open-Loop-Playback, Sec. V-B und Table IV (Zeile Playback): Laeufe 001-006 in
%      data/hardware/playback_runs/. Endeffektor per FK der Gen3-URDF (end_effector_link),
%      Gelenkwinkel mit Wrapping auf (-180, 180] deg.
%        - "integriert": dq_cmd ueber die tatsaechliche Schleifenzeit integriert (Servo-Folge)
%        - "Plan": dq_cmd ueber die geplante Zeit t_ref integriert (geplanter Takt)
%        - Gelenk an der Grenze: gemessene Position bis auf 1 deg an der Datenblattgrenze
%          (J2 128.9, J4 147.8, J6 120.3 deg)
%        - erste Gelenkabweichung: gemessen gegen integriert > 1 deg
%   2) Werkzeug-Offset der Kortex-tool_pose (A42, Sec. VI-C): Fit ueber die Set-Point-Laeufe
%      068-073, Modell tool_pose = R_z' * (FK_URDF + R_ee * t_tool) mit Basisversatz b.
% Die Zahlen ohne FK stehen in legacy_numbers.py.
%
% Aufruf: setup_project; run(fullfile(sk_path(), 'evaluation', 'paper', 'legacy_fk_numbers.m'))
% Schreibt evaluation/paper/legacy_fk_numbers.csv (key, value, unit, files, note).

rows = cell(0, 5);
outFile = fullfile(sk_path(), 'evaluation', 'paper', 'legacy_fk_numbers.csv');

%% 1) Playback
gen3 = importrobot(sk_path('robot', 'GEN3-7DOF-VISION_ARM_URDF_V12.urdf'));
gen3.DataFormat = 'row';
fk = @(q) tform2trvec(getTransform(gen3, deg2rad(q), 'end_effector_link'));
wrap = @(x) mod(x + 180, 360) - 180;
f = dir(sk_path('data', 'hardware', 'playback_runs', 'run_00*.mat'));
fprintf('Playback: %d Laeufe\n', numel(f));
for k = 1:numel(f)
    L = load(fullfile(f(k).folder, f(k).name));
    d = L.data; m = L.meta;
    t = d.t(:); tr = d.t_ref(:); q = d.q_measured; dq = d.dq_cmd; n = numel(t);
    qa = zeros(n, 7); qa(1, :) = q(1, :); qp = qa;
    for i = 2:n
        qa(i, :) = qa(i - 1, :) + dq(i - 1, :) * (t(i) - t(i - 1));
        qp(i, :) = qp(i - 1, :) + dq(i - 1, :) * (tr(i) - tr(i - 1));
    end
    ea = zeros(n, 1); ep = zeros(n, 1);
    for i = 1:n
        pm = fk(q(i, :));
        ea(i) = norm(pm - fk(qa(i, :)));
        ep(i) = norm(pm - fk(qp(i, :)));
    end
    je = abs(wrap(q - qa));
    qw = abs(wrap(q));
    limHw = [128.9 147.8 120.3];                         % Gen3-Datenblatt, J2 J4 J6 [deg]
    atLimit = [2 4 6];
    atLimit = atLimit(max(qw(:, [2 4 6]), [], 1) >= limHw - 1);
    i1 = find(max(je, [], 2) > 1, 1);
    id = f(k).name(1:7);
    src = ['playback_runs/' f(k).name];
    rows = add(rows, [id '.timeScale'], m.timeScale, '', src, '');
    rows = add(rows, [id '.duration'], t(end) - t(1), 's', src, 'erster bis letzter Schritt');
    rows = add(rows, [id '.t_end'], t(end), 's', src, 'Zeitstempel des letzten Schritts');
    rows = add(rows, [id '.loop_median'], 1e3 * median(diff(t)), 'ms', src, '');
    rows = add(rows, [id '.ee_vs_integrated_rms'], 1e3 * sqrt(mean(ea .^ 2)), 'mm', src, 'Servo-Folge');
    rows = add(rows, [id '.ee_vs_plan_rms'], 1e3 * sqrt(mean(ep .^ 2)), 'mm', src, 'geplanter Takt');
    rows = add(rows, [id '.joints_at_limit'], strjoin(compose('J%d', atLimit), ' '), '', src, ...
               'gemessene Position bis auf 1 deg an der Datenblattgrenze');
    rows = add(rows, [id '.max_abs_J2_J4_J6'], strjoin(compose('%.1f', max(qw(:, [2 4 6]), [], 1)), ' '), ...
               'deg', src, '');
    if ~isempty(i1)
        rows = add(rows, [id '.t_first_joint_dev'], t(i1) - t(1), 's', src, '');
        rows = add(rows, [id '.ee_vs_plan_at_first_joint_dev'], 1e3 * ep(i1), 'mm', src, '');
    end
end

%% 2) Werkzeug-Offset der Kortex-tool_pose (A42)
sk = importrobot(sk_path('robot', 'SpaceKinova.urdf'));
sk.DataFormat = 'row';
A = []; y = []; src = {};
for r = 68:73
    fr = dir(sk_path('data', 'hardware', 'runs', sprintf('run_%03d_*.mat', r)));
    S = load(fullfile(fr(1).folder, fr(1).name));
    qd = S.data.q_measured; ek = S.data.ee_kortex;
    src{end + 1} = ['runs/' fr(1).name]; %#ok<SAGROW>
    for i = 1:max(1, floor(size(qd, 1) / 40)):size(qd, 1)
        Ti = getTransform(sk, deg2rad(qd(i, :)), 'kinova_end_effector_link');
        A = [A; Ti(1:3, 1:3) eye(3)]; %#ok<AGROW>
        y = [y; ek(i, :).' - Ti(1:3, 4)]; %#ok<AGROW>
    end
end
x = A \ y;
res = vecnorm(reshape(y - A * x, 3, []));
srcAll = strjoin(src, ';');
rows = add(rows, 'tool_offset_z', x(3), 'm', srcAll, 'Kortex-Werkzeugpunkt im EE-Frame, z-Anteil');
rows = add(rows, 'tool_offset_xy', norm(x(1:2)), 'm', srcAll, '');
rows = add(rows, 'base_offset_z', x(6), 'm', srcAll, 'Kortex-Frame minus URDF-Frame');
rows = add(rows, 'tool_fit_rms', 1e3 * sqrt(mean(res .^ 2)), 'mm', srcAll, sprintf('%d Punkte', numel(res)));
rows = add(rows, 'tool_fit_max', 1e3 * max(res), 'mm', srcAll, '');

%% Schreiben
fid = fopen(outFile, 'w');
fprintf(fid, 'key,value,unit,files,note\n');
for i = 1:size(rows, 1)
    v = rows{i, 2};
    if isnumeric(v), v = sprintf('%.6g', v); end
    fprintf(fid, '%s,%s,%s,%s,%s\n', rows{i, 1}, v, rows{i, 3}, rows{i, 4}, rows{i, 5});
end
fclose(fid);
fprintf('%d Werte nach %s\n', size(rows, 1), outFile);

function rows = add(rows, key, value, unit, files, note)
rows(end + 1, :) = {key, value, unit, files, note};
if isnumeric(value), vs = sprintf('%.4g', value); else, vs = value; end
fprintf('  %-40s %s %s  %s\n', key, vs, unit, note);
end

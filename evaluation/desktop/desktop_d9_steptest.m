function T = desktop_d9_steptest()
%DESKTOP_D9_STEPTEST  Sprungtest des Gen3 auswerten (Desktop-Plan D9, Aussage H2 in Sec. V-C).
%   T = desktop_d9_steptest()
%
%   Log data/hardware/kinova_velocity_test_log.mat aus hardware/playback/kinova_test.m: jedes Gelenk nacheinander
%   mit 5 Grad/s (beide Richtungen, je 1,5 s), Soll-Rate 100 Hz, Befehl ueber SendJointSpeedCommand.
%   Aus den gemessenen Winkeln wird die Geschwindigkeit je Sprung rekonstruiert (zentrale Differenz). Je Sprung:
%     gain      mittlere Geschwindigkeit in der zweiten Haelfte des Sprungs / Befehl
%     t50, t90  Zeit vom Befehlsbeginn bis 50 % bzw. 90 % des Befehls
%     lagPath   Phasenverzug bei der Frequenz der Halbkreisbahn (1/17 Hz bei 8,5 s pro Halbkreis), abgeschaetzt
%               aus einer Totzeit von t50 (360 * f * t50)
%   Die Schleife lief mit etwa 50 ms statt 10 ms pro Schritt. t50 und t90 sind deshalb nur auf etwa 50 ms genau.
%   Ergebnis: data/simulation/desktop/D9_steptest.csv

setup_project;
S = load(sk_path('data', 'hardware', 'kinova_velocity_test_log.mat'));
L = S.log;
t = L.t(:);
q = unwrap(deg2rad(L.q)) * 180 / pi;           % Grad, ohne 360-Spruenge
cmd = L.dq_cmd;
dt = diff(t);
fprintf('Log: %d Proben, %.1f s, Schritt Median %.1f ms (Soll %.0f ms), Befehl max %.1f Grad/s\n', numel(t), ...
    sum(dt(dt > 0)), 1000 * median(dt(dt > 0)), 1000 / S.cfg.rateHz, max(abs(cmd(:))));
fprintf(['Die Zeitachse beginnt in jedem Abschnitt neu (%d Ruecksprunge). ' ...
    'Schritte 40-60 ms: %d, 60-100 ms: %d, ueber 100 ms: %d\n'], ...
    sum(dt < 0), sum(dt >= 0.04 & dt < 0.06), sum(dt >= 0.06 & dt < 0.1), sum(dt >= 0.1));


rows = {};
for j = 1:size(cmd, 2)
    on = abs(cmd(:, j)) > 1e-6;
    d = diff([0; on; 0]);
    s0 = find(d == 1); s1 = find(d == -1) - 1;
    for k = 1:numel(s0)
        idx = s0(k):s1(k);
        c = cmd(idx(1), j);
        tt = t(idx) - t(idx(1));
        vv = gradient(q(idx, j), t(idx));          % nur innerhalb des Sprungs, die Zeitachse springt zwischen Abschnitten
        half = tt >= tt(end) / 2;
        gain = mean(vv(half)) / c;
        t50 = tt(find(vv / c >= 0.5, 1));
        t90 = tt(find(vv / c >= 0.9, 1));
        if isempty(t50), t50 = NaN; end
        if isempty(t90), t90 = NaN; end
        rows{end + 1} = struct('joint', j, 'cmd_degps', c, 'duration_s', tt(end), 'gain', gain, ...
            't50_ms', 1000 * t50, 't90_ms', 1000 * t90, 'lagPath_deg', 360 / 17 * t50, ...
            'ripple_degps', std(vv(half))); %#ok<AGROW>
    end
end
T = struct2table([rows{:}]);
disp(T);
fprintf('Verstaerkung: Median %.3f (%.3f bis %.3f) | t50 Median %.0f ms | t90 Median %.0f ms\n', median(T.gain), ...
    min(T.gain), max(T.gain), median(T.t50_ms, 'omitnan'), median(T.t90_ms, 'omitnan'));
writetable(T, sk_path('data', 'simulation', 'desktop', 'D9_steptest.csv'));
end

function kinova_move_joints(apiHandle, q_target_rad, cfg)
%KINOVA_MOVE_JOINTS  Faehrt per ReachJointAngles in eine Gelenkpose und wartet.
%   Aufruf wie playback_variants.m und deploy_tracking_v24 (dort erprobt).
%   Wartet, bis alle Gelenke 5 Zyklen lang innerhalb cfg.homingTol_deg liegen,
%   danach cfg.homingSettle_s Ruhezeit. Fehler bei Fault oder Zeitueberschreitung.
target_deg = rad2deg(q_target_rad(:)).';
errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, int32(0), 0, 0, target_deg);
if errCode ~= 0
    error('kinova_move_joints:reach', 'ReachJointAngles fehlgeschlagen (errorCode=%d).', errCode);
end
tStart = tic; stable = 0;
while toc(tStart) < cfg.homingTimeout_s
    st = kinova_read_state(apiHandle, cfg);
    if st.fault, error('kinova_move_joints:fault', 'Fault waehrend der Anfahrt.'); end
    err_deg = rad2deg(mod(st.q_rad - q_target_rad(:) + pi, 2*pi) - pi);
    if all(abs(err_deg) < cfg.homingTol_deg)
        stable = stable + 1;
        if stable >= 5, break; end
    else
        stable = 0;
    end
    pause(0.1);
end
if stable < 5
    error('kinova_move_joints:timeout', 'Pose nicht innerhalb von %.0f s erreicht.', cfg.homingTimeout_s);
end
pause(cfg.homingSettle_s);
end

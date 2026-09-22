function errCode = kinova_send_deg(apiHandle, dq_deg, cfg)
%KINOVA_SEND_DEG  Sendet ein Joint-Speed-Kommando in deg/s (genau diese Werte werden geloggt).
errCode = kortexApiMexInterface('SendJointSpeedCommand', ...
    apiHandle, cfg.speedCmdDuration, dq_deg(:).', uint32(cfg.nJ));
end

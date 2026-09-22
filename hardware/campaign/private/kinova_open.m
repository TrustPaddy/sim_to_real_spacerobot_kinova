function apiHandle = kinova_open(cfg)
%KINOVA_OPEN  Oeffnet die MEX-Session zum Kinova Gen3 (wie V2.3 kinovaOpen).
[errCode, apiHandle, ~] = kortexApiMexInterface( ...
    'CreateRobotApisWrapper', ...
    cfg.kinovaIP, cfg.kinovaUser, cfg.kinovaPassword, ...
    cfg.sessionTimeoutMs, cfg.controlTimeoutMs);
if errCode ~= 0
    error('kinova_open:create', 'CreateRobotApisWrapper fehlgeschlagen (errorCode=%d).', errCode);
end
fprintf('Verbunden mit Kinova Gen3 @ %s (apiHandle=%d)\n', cfg.kinovaIP, apiHandle);
end

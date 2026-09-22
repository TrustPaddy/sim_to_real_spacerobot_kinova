function kinova_safe_shutdown(apiHandle, nJ)
%KINOVA_SAFE_SHUTDOWN  Zero-Velocity, StopAction und Session schliessen (wie V2.3 safeShutdown).
try
    fprintf('\n[safeShutdown] Sende Zero-Velocity...\n');
    kortexApiMexInterface('SendJointSpeedCommand', apiHandle, 0, zeros(1, nJ), uint32(nJ));
    pause(0.05);
catch
end
try
    kortexApiMexInterface('StopAction', apiHandle);
catch
end
try
    kortexApiMexInterface('DestroyRobotApisWrapper', apiHandle);
    fprintf('[safeShutdown] MEX-Session geschlossen.\n');
catch
end
end

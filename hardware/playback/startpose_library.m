function poses = startpose_library()
    % Pose mit der der Agent trainiert wurde
    poses.training = [0 0 0 0 0 0 0];
    
    % Andere Pose, NUR fuer den Open-Loop-Hardware-Test in B1.
    % Wird NICHT fuer den Agenten verwendet.
    poses.alt = [0 15 180 230 0 55 90];   

    poses.elbow_bent = [0 15 180 230 0 55 90]; % Beispiel "elbow_bent"
end
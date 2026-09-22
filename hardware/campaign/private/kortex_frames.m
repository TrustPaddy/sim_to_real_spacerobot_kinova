function F = kortex_frames(robot_rbt)
%KORTEX_FRAMES  Beziehung zwischen URDF-Frame (Training) und Kortex-Frames.
%   F.T_urdf_from_real  4x4, Kortex-Basis-Frame -> URDF-Frame (base_link).
%                       Im URDF sitzt kinova_base_link 0.5 m ueber base_link.
%   F.toolOffset        3x1, Kortex-Werkzeugpunkt (tool_pose) im Frame von
%                       kinova_end_effector_link [m]. Im Kortex-Tool ist der
%                       Greifer eingetragen, tool_pose liegt deshalb 0.121 m
%                       entlang der Werkzeug-z-Achse vor dem Trainingspunkt.
%                       Fit ueber die Set-Point-Laeufe 068-073 (271 Punkte,
%                       Restfehler RMS 3 mm, max 10 mm), siehe MESSPLAN.md.
%   F.gripperPoints     3xN, Punkte auf dem Greifer im EE-Frame fuer die
%                       Hoehenpruefung (Greifer fehlt in der URDF).
F.T_urdf_from_real = getTransform(robot_rbt, homeConfiguration(robot_rbt), 'kinova_base_link');
F.toolOffset    = [0; 0; 0.121];
F.gripperPoints = [0 0 0; 0 0 0.10; 0 0 0.17].';
F.note = ['toolOffset aus Fit der Set-Point-Laeufe 068-073 (271 Punkte, RMS 3 mm). ' ...
          'Greifer im Kortex-Tool konfiguriert.'];
end

function e = desktop_v21_eori(q)
%DESKTOP_V21_EORI  e_ori-Eintrag der Beobachtung, wie ihn das Deploy-Skript V2.1 gebildet hat (Befund A22).
%   e = desktop_v21_eori(q)   q: 7x1 Gelenkwinkel [rad]
%
%   V2.1 setzt R_err = ee_rot' und nimmt davon den schiefsymmetrischen Anteil. Dort steht also die
%   Endeffektor-Orientierung, im Training dagegen der Orientierungsfehler der Basis (auf fester Basis null).
%   Wird aus dem Simulink-Block "Obs Transform" als extrinsische Funktion aufgerufen.

persistent robot
if isempty(robot)
    robot = importrobot(sk_path('robot', 'SpaceKinova.urdf'));
    robot.DataFormat = 'row';
end
T = getTransform(robot, q(:).', 'kinova_end_effector_link');
R_err = T(1:3, 1:3).';
e = [R_err(3,2) - R_err(2,3); R_err(1,3) - R_err(3,1); R_err(2,1) - R_err(1,2)] / 2;
end

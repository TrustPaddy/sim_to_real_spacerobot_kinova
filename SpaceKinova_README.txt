How to generate SpaceKinova.urdf
===============================

1) Put your Kinova URDF somewhere, e.g.
   kinova_gen3.urdf

2) Run in MATLAB:
   out = make_spacekinova_urdf("kinova_gen3.urdf","SpaceKinova.urdf", ...
       "Prefix","kinova_", ...
       "MountXYZ",[0 0 0.5], ...
       "MountRPY",[0 0 0]);

3) Test import:
   robot = importrobot("SpaceKinova.urdf");
   robot.DataFormat = "row";
   showdetails(robot);

Notes
-----
- The cube base defaults follow your report's SpaceRobot.urdf:
  box size 1x1x1 m, mass 65 kg, inertia diag [8 8 8].
- If your Kinova URDF already has unique names, you can set Prefix="".
- If the Kinova base ends up floating inside the cube, adjust MountXYZ.

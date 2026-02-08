# Sim-to-Real SpaceRobot Kinova

Reinforcement-Learning-basierte Trajektorienverfolgung fuer einen 7-DOF Kinova-Roboterarm auf einer frei schwebenden Wuerfelplattform (Weltraum-Szenario). Das Projekt nutzt einen Sim-to-Real-Ansatz: Training in MATLAB/Simulink, Uebertragung auf echte Hardware via ROS.

## Ueberblick

| Komponente | Beschreibung |
|---|---|
| **Roboter** | Kinova Gen3 (7 DOF) auf Wuerfelbasis (65 kg, 1x1x1 m) |
| **RL-Algorithmus** | PPO (Proximal Policy Optimization) |
| **Simulation** | Simulink + Simscape Multibody |
| **Hardware-Interface** | ROS 1 (kortex_driver) |
| **Freiheitsgrade** | 13 total (6 DOF Basis + 7 DOF Arm) |

## Projektstruktur

```
Sim_to_real_spacerobot_kinova/
  SpaceKinovaDynamic_PPO.m    - RL-Training (PPO) in Simulink
  kinova_test.m               - Hardware-Velocity-Test via ROS
  make_spacekinova_urdf.m     - URDF-Generierung (Wuerfelbasis + Kinova)
  SpaceRobot.slx              - Simulink-Modell mit Physik-Simulation
  SpaceKinova_TEMPLATE.urdf   - URDF-Template
  SpaceKinova_README.txt      - URDF-Generierungsanleitung
```

## Voraussetzungen

**MATLAB R2020b oder neuer** mit folgenden Toolboxen:

- Reinforcement Learning Toolbox
- Robotics System Toolbox
- Simulink
- Simscape / Simscape Multibody
- ROS Toolbox (fuer Hardware-Tests)

**Hardware (fuer Sim-to-Real):**

- Kinova Gen3 7-DOF Roboterarm
- ROS 1 mit `kortex_driver`

## Schnellstart

### Schritt 1: URDF generieren

Eine Kinova-URDF-Datei (z.B. `kinova_gen3.urdf`) muss vorhanden sein. Dann:

```matlab
make_spacekinova_urdf("kinova_gen3.urdf", "SpaceKinova.urdf", ...
    "Prefix", "kinova_", ...
    "MountXYZ", [0 0 0.5]);
```

Ergebnis pruefen:

```matlab
robot = importrobot("SpaceKinova.urdf");
robot.DataFormat = "row";
showdetails(robot);
show(robot);
```

### Schritt 2: RL-Agent trainieren (Simulation)

```matlab
SpaceKinovaDynamic_PPO
```

Das Skript:
1. Laedt die generierte URDF und berechnet die Referenztrajektorie (Kreis, r=0.4 m)
2. Loest die inverse Kinematik fuer gewuenschte Gelenkwinkel
3. Erstellt die RL-Umgebung (27D Observation, 7D Action)
4. Trainiert einen PPO-Agenten (max. 1000 Episoden)
5. Speichert den trainierten Agenten in `savedAgents_spacekinova/`

**Wichtig:** Das Simulink-Modell muss vorher angepasst sein:
- Robot-Subsystem nutzt `SpaceKinova.urdf`
- `RL_Agent`-Block existiert unter `mdl/RL_Agent`
- Reward/Done und Collision-Monitor sind auf 7 Joints ausgelegt

### Schritt 3: Hardware-Test (Kinova via ROS)

```matlab
kinova_test
```

Testet jeden Joint einzeln mit kleinen Geschwindigkeitskommandos. Sicherheitsfeatures:
- Velocity-Saettigung (max 0.25 rad/s)
- Watchdog-Timeout (0.25 s)
- Automatischer Stopp bei CTRL+C (`onCleanup`)
- Manuell bestaetigt pro Joint (ENTER)

**Konfiguration anpassen** (in `kinova_test.m`, Zeilen 13-45):

```matlab
cfg.ns        = "/my_gen3";            % Namespace
cfg.cmdTopic  = cfg.ns + "/in/joint_speeds";  % Velocity-Topic
cfg.jointNames = ["joint_1", ..., "joint_7"]; % Joint-Namen
```

## RL-Umgebung Details

### Observation Space (27 Dimensionen)

| Signal | Dimension | Beschreibung |
|---|---|---|
| Position Error | 3 | Endeffector-Positionsfehler [m] |
| Velocity Error | 3 | Endeffector-Geschwindigkeitsfehler [m/s] |
| Joint Angles (q) | 7 | Gelenkwinkel [rad] |
| Joint Velocities (dq) | 7 | Gelenkgeschwindigkeiten [rad/s] |
| Base Linear Velocity | 3 | Basisgeschwindigkeit [m/s] |
| Base Angular Velocity | 3 | Basis-Drehgeschwindigkeit [rad/s] |
| Orientation Error | 3 | Orientierungsfehler [rad] |

### Action Space (7 Dimensionen)

Gelenkgeschwindigkeitskommandos `dq_cmd`, begrenzt auf +/- 0.8 rad/s.

### Referenztrajektorie

Kreisbahn in der x-y-Ebene:
- Radius: 0.4 m
- Periode: 8.5 s
- Konstante z-Hoehe

## URDF-Generierung

`make_spacekinova_urdf` kombiniert eine Kinova-URDF mit einer Wuerfelbasis:

| Parameter | Default | Beschreibung |
|---|---|---|
| `Prefix` | `"kinova_"` | Praefix fuer Link-/Joint-Namen |
| `MountXYZ` | `[0 0 0.5]` | Montageposition auf der Basis |
| `MountRPY` | `[0 0 0]` | Montageorientierung |
| `BaseSize` | `[1 1 1]` | Wuerfelabmessungen [m] |
| `BaseMass` | `65` | Basismasse [kg] |
| `BaseInertiaDiag` | `[8 8 8]` | Traegheitsmomente [kg*m^2] |

## Troubleshooting

**Roboter reagiert nicht auf ROS-Kommandos:**
- Namespace und Topic pruefen (`rostopic list`)
- Message-Typ kontrollieren (`kortex_driver/JointSpeeds` vs. `trajectory_msgs/JointTrajectory`)
- JointIdentifier-Basis: 0-basiert vs. 1-basiert (in `sendJointVelocity` anpassen)

**IK-Konvergenz-Probleme:**
- `ikWeights` anpassen (Position vs. Orientierung)
- Andere `homeConfiguration` als Seed verwenden

**Simulink-Fehler beim Training:**
- Pruefen, ob alle `assignin`-Variablen im Base Workspace vorhanden sind
- `RL_Agent`-Block-Pfad muss mit `cfg.agentBlk` uebereinstimmen

# Sim-to-Real SpaceRobot Kinova Gen3

Reinforcement-Learning-basierte Trajektorienverfolgung fuer einen Kinova Gen3 7-DOF Roboterarm auf einer frei schwebenden Wuerfelplattform (Weltraum-Szenario). Das Projekt nutzt einen Sim-to-Real-Ansatz: Training in MATLAB/Simulink, Uebertragung auf echte Hardware via ROS.

## Ueberblick

| Komponente | Beschreibung |
|---|---|
| **Roboter** | Kinova Gen3 7-DOF auf Wuerfelbasis (65 kg, 1x1x1 m) |
| **RL-Algorithmus** | PPO (Proximal Policy Optimization) |
| **Simulation** | Simulink + Simscape Multibody |
| **Hardware-Interface** | ROS 1 (kortex_driver, Noetic) |
| **Freiheitsgrade** | 13 total (6 DOF Basis + 7 DOF Arm) |
| **Agent-Rate** | 40 Hz (= Kinova High-Level Servo Rate) |

## Kinova Gen3 7-DOF Spezifikationen

| Joint | Typ | Positionslimit [rad] | Geschwindigkeit [rad/s] | Drehmoment nominal [Nm] | Drehmoment peak [Nm] |
|-------|-----|---------------------|------------------------|------------------------|---------------------|
| J1 | continuous | +/-2pi (Software) | 1.3963 | 32 | 74 |
| J2 | revolute | +/-2.41 | 1.3963 | 32 | 74 |
| J3 | continuous | +/-2pi (Software) | 1.3963 | 32 | 74 |
| J4 | revolute | +/-2.66 | 1.3963 | 32 | 74 |
| J5 | revolute | +/-2.23 | 1.2218 | 13 | 34 |
| J6 | revolute | +/-2.01 | 1.2218 | 13 | 34 |
| J7 | continuous | +/-2pi (Software) | 1.2218 | 13 | 34 |

Quellen: [ros_kortex URDF](https://github.com/Kinovarobotics/ros_kortex), Kinova Gen3 User Guide

## Projektstruktur

```
Sim_to_real_spacerobot_kinova/
  SpaceKinovaDynamic_PPO.m    - RL-Training (PPO) in Simulink
  deploy_agent_kinova.m       - Deployment: Agent auf echtem Kinova Gen3
  kinova_test.m               - Hardware-Velocity-Test via ROS
  make_spacekinova_urdf.m     - URDF-Generierung (Wuerfelbasis + Kinova)
  SpaceRobot.slx              - Simulink-Modell mit Physik-Simulation
  SpaceKinova_TEMPLATE.urdf   - URDF-Referenz-Template
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
- ROS 1 Noetic mit `kortex_driver`
- Ethernet-Verbindung zum Kinova (Standard-IP: 192.168.1.10)

## Schnellstart

### Schritt 1: URDF generieren

Eine Kinova-URDF-Datei (z.B. `kinova_gen3.urdf` aus dem `ros_kortex` Paket) muss vorhanden sein. Dann:

```matlab
make_spacekinova_urdf("kinova_gen3.urdf", "SpaceKinova.urdf", ...
    "Prefix", "kinova_", ...
    "MountXYZ", [0 0 0.5]);
```

Die Funktion validiert die generierte URDF automatisch und zeigt Anzahl Bodies/DOF an.

### Schritt 2: RL-Agent trainieren (Simulation)

```matlab
SpaceKinovaDynamic_PPO
```

Das Skript:
1. Laedt die generierte URDF und berechnet die Referenztrajektorie (Kreis, r=0.4 m)
2. Loest die inverse Kinematik fuer gewuenschte Gelenkwinkel
3. Erstellt die RL-Umgebung (29D Observation, 7D Action)
4. Trainiert einen PPO-Agenten (max. 1000 Episoden, 40 Hz)
5. Speichert den besten Agenten automatisch in `savedAgents_spacekinova/`

**Wichtig:** Das Simulink-Modell muss vorher angepasst sein:
- Robot-Subsystem nutzt `SpaceKinova.urdf`
- `RL_Agent`-Block existiert unter `mdl/RL_Agent` mit Sample Time = 0.025 s
- Saturation-Bloecke nutzen die 7x1 Vektoren `dq_max` und `tau_max` aus dem Workspace
- Reward/Done und Collision-Monitor sind auf 7 Joints ausgelegt

### Schritt 3: Hardware-Test (Kinova via ROS)

```matlab
kinova_test
```

Testet jeden Joint einzeln mit kleinen Geschwindigkeitskommandos:
- Per-Joint Velocity-Saettigung (20% der echten Limits)
- Watchdog-Timeout (100 ms)
- Automatischer Stopp bei CTRL+C (`onCleanup`)
- Manuell bestaetigt pro Joint (ENTER)

### Schritt 4: Agent auf Hardware deployen

```matlab
deploy_agent_kinova
```

Workflow:
1. Trainierter Agent wird automatisch aus `savedAgents_spacekinova/` geladen
2. **Dry-Run Modus (Standard):** Kommandos werden angezeigt, NICHT gesendet
3. Dry-Run Ausgaben pruefen: Geschwindigkeiten plausibel? Positionsfehler sinkt?
4. `cfg.dryRun = false` setzen fuer echte Ausfuehrung
5. Agent steuert Kinova Gen3 in Echtzeit (40 Hz)
6. Log wird automatisch gespeichert mit Tracking-Metriken

## Sicherheitshinweise (Sim-to-Real)

**WARNUNG:** Vor der Ausfuehrung auf echter Hardware:

1. **Geschwindigkeitslimits:** Der RL-Agent arbeitet mit 70% der echten Kinova-Limits (Sicherheitsfaktor 0.7). NIEMALS den Sicherheitsfaktor auf >0.8 erhoehen.
2. **Dry-Run zuerst:** `deploy_agent_kinova.m` startet im Dry-Run-Modus. Erst nach Verifizierung der Ausgaben `cfg.dryRun = false` setzen.
3. **E-Stop:** Immer den Not-Aus-Schalter bereithalten.
4. **Arbeitsraum freiraeumen:** Keine Objekte im Arbeitsbereich des Roboters.
5. **Frequenz-Match:** Der Agent laeuft bei 40 Hz, passend zum Kinova High-Level Servo.
6. **Basis-Geschwindigkeit:** Im Training ist die Basis frei schwebend. Auf echter Hardware ist die Basis fest -> v_base = w_base = [0,0,0]. Der Agent lernt dies durch Domain Randomization.
7. **Ersttest:** Immer zuerst `kinova_test.m` ausfuehren, um ROS-Verbindung zu validieren.
8. **Soft-Limits:** Das Deployment-Skript bremst automatisch, wenn ein Gelenk sich 10 Grad vor seinem Limit befindet.

## RL-Umgebung Details

### Observation Space (29 Dimensionen)

| Signal | Dimension | Beschreibung |
|---|---|---|
| Position Error | 3 | Endeffector-Positionsfehler [m] |
| Velocity Error | 3 | Endeffector-Geschwindigkeitsfehler [m/s] |
| Joint Angles (q) | 7 | Gelenkwinkel [rad], per-Joint Limits |
| Joint Velocities (dq) | 7 | Gelenkgeschwindigkeiten [rad/s], per-Joint Limits |
| Base Linear Velocity | 3 | Basisgeschwindigkeit [m/s] |
| Base Angular Velocity | 3 | Basis-Drehgeschwindigkeit [rad/s] |
| Orientation Error | 3 | Orientierungsfehler [rad] |

### Action Space (7 Dimensionen)

Gelenkgeschwindigkeitskommandos `dq_cmd`, per-Joint begrenzt auf 70% der Kinova-Limits:
- J1-J4: +/- 0.977 rad/s (grosse Aktuatoren)
- J5-J7: +/- 0.855 rad/s (kleine Aktuatoren)

### ROS-Interface (kortex_driver)

| Parameter | Wert |
|---|---|
| Velocity-Topic | `/my_gen3/in/joint_velocity` |
| Message-Typ | `kortex_driver/Base_JointSpeeds` |
| Joint Identifiers | 0-basiert (0..6) |
| Kontrollrate | 40 Hz (High-Level Servo) |

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
| `BaseInertiaDiag` | `[10.833 10.833 10.833]` | Traegheitsmomente [kg*m^2] (korrekt fuer 65kg Wuerfel) |

## Simulink-Modell Anpassungen

Das Simulink-Modell `SpaceRobot.slx` muss folgendermassen konfiguriert sein:

1. **RL Agent Block:** Sample Time = `0.025` (oder Variable `Ts_agent`)
2. **Saturation (dq_cmd):** Upper/Lower Limit = `dq_max` / `-dq_max` (7x1 Vektor aus Workspace)
3. **Saturation (Torque):** Upper/Lower Limit = `tau_max` / `-tau_max` (7x1 Vektor aus Workspace)
4. **Gelenkgrenzen:** `qLim_lower` und `qLim_upper` (7x1 Vektoren) statt skalarem `qLim_abs`
5. **URDF-Referenz:** `SpaceKinova.urdf` (generiert durch `make_spacekinova_urdf`)

## Troubleshooting

**Roboter reagiert nicht auf ROS-Kommandos:**
- Namespace pruefen: `rostopic list | grep my_gen3`
- Topic: `/my_gen3/in/joint_velocity` (NICHT `/in/joint_speeds`)
- Message-Typ: `kortex_driver/Base_JointSpeeds` (Noetic)
- Joint IDs sind 0-basiert (0..6)

**IK-Konvergenz-Probleme:**
- `ikWeights` anpassen (Position vs. Orientierung)
- Andere `homeConfiguration` als Seed verwenden

**Simulink-Fehler beim Training:**
- Pruefen, ob alle `assignin`-Variablen im Base Workspace vorhanden sind
- `RL_Agent`-Block-Pfad muss mit `cfg.agentBlk` uebereinstimmen
- Saturation-Bloecke muessen 7x1 Vektoren akzeptieren (nicht Skalare)

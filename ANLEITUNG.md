# Schritt-fuer-Schritt Anleitung: Vom Training zum echten Kinova Gen3

Diese Anleitung fuehrt dich durch den kompletten Workflow — von der URDF-Generierung ueber das RL-Training bis zur Ansteuerung deines echten Kinova Gen3 mit den trainierten Geschwindigkeitswerten.

> **Dein Ziel:** Ein PPO-Agent lernt in der Simulation, den Endeffector des Kinova Gen3 entlang einer Kreisbahn zu fuehren. Die gelernten Gelenkgeschwindigkeitskommandos (`dq_cmd`) werden dann auf den echten Roboter uebertragen.

---

## Uebersicht: Die 5 Phasen

```
Phase 1: URDF generieren          (~1 Min)
    |
Phase 2: Simulink-Modell anpassen (~5 Min, einmalig)
    |
Phase 3: PPO-Agent trainieren     (~1-3 Stunden)
    |
Phase 4: Hardware-Test via ROS    (~10 Min)
    |
Phase 5: Agent auf Kinova deployen (~10 Min)
```

---

## Was du brauchst

### Software
- **MATLAB R2020b** oder neuer
- **Toolboxen:** Reinforcement Learning, Robotics System, Simulink, Simscape Multibody, ROS Toolbox
- **ROS 1 Noetic** auf einem separaten Rechner oder lokal (mit `kortex_driver`)

### Hardware
- Kinova Gen3 7-DOF (fest montiert)
- Ethernet-Verbindung zum Kinova (Standard-IP: `192.168.1.10`)
- Not-Aus-Schalter griffbereit

### Dateien
- **Kinova Gen3 URDF** aus dem [`ros_kortex`](https://github.com/Kinovarobotics/ros_kortex) Paket
  - Pfad im Paket: `kortex_description/arms/gen3/7dof/urdf/gen3.urdf` (oder die `.xacro` vorher konvertieren)
  - Falls du `ros_kortex` installiert hast: `roscd kortex_description && ls arms/gen3/7dof/urdf/`

---

## Phase 1: URDF generieren

### Was passiert hier?
Dein Kinova Gen3 sitzt in der Simulation auf einer frei schwebenden Wuerfelplattform (65 kg, 1x1x1 m) — das simuliert ein Weltraum-Szenario. `make_spacekinova_urdf.m` nimmt die Original-Kinova-URDF und kombiniert sie mit diesem Wuerfel zu einer einzigen URDF-Datei.

### So gehst du vor

```matlab
% 1. Kinova URDF in dein Projektverzeichnis kopieren
%    z.B. als "kinova_gen3.urdf"

% 2. SpaceKinova-URDF erzeugen
make_spacekinova_urdf("kinova_gen3.urdf", "SpaceKinova.urdf");
```

### Was das Skript macht
1. Liest die Kinova-URDF ein
2. Fuegt allen Link-/Joint-Namen den Prefix `kinova_` hinzu (verhindert Namenskollisionen mit `base_link`)
3. Erstellt den Wuerfel (`base_link`) mit korrekten Traegheitsmomenten (10.833 kg*m²)
4. Verbindet den Wuerfel ueber ein festes Gelenk mit dem Kinova-Arm
5. Schreibt die kombinierte URDF nach `SpaceKinova.urdf`
6. **Validiert automatisch:** Zeigt Anzahl Bodies und Freiheitsgrade an

### Ergebnis pruefen

```matlab
robot = importrobot("SpaceKinova.urdf");
robot.DataFormat = "row";
showdetails(robot)   % Sollte 7 DOF anzeigen (plus fixed joints)
show(robot);         % 3D-Visualisierung: Wuerfel mit Kinova-Arm oben drauf
```

Du solltest einen blauen Wuerfel sehen, auf dem der Kinova-Arm montiert ist.

### Moegliche Probleme
| Problem | Loesung |
|---------|---------|
| "URDF not found" | Pfad zur Kinova-URDF pruefen, Datei muss im gleichen Ordner oder mit vollem Pfad angegeben sein |
| Weniger als 7 DOF | Kinova-URDF ist evtl. die 6-DOF Version — du brauchst die 7-DOF Variante |
| Arm schwebt in der Luft | `MountXYZ` anpassen: `[0 0 0.5]` setzt den Arm oben auf den 1m-Wuerfel |

---

## Phase 2: Simulink-Modell einrichten

### Was passiert hier?
Das Simulink-Modell (`SpaceRobot.slx`) ist die Physik-Simulation, in der der RL-Agent trainiert wird. Es simuliert die Dynamik des Wuerfels + Kinova-Arms und berechnet in jedem Zeitschritt die Observation (was der Agent "sieht") und den Reward (wie gut er ist).

### Anpassungen im Simulink-Modell

Oeffne `SpaceRobot.slx` in Simulink und passe folgendes an:

**1. RL Agent Block** (Pfad: `SpaceKinova/RL_Agent`)
- Doppelklick auf den Block
- **Sample Time** von `0.1` auf **`0.025`** aendern (= 40 Hz, passend zum Kinova)

**2. Saturation-Bloecke fuer Gelenkgeschwindigkeit** (`dq_cmd`)
- Upper Limit: `dq_max` (liest 7x1 Vektor aus dem Workspace)
- Lower Limit: `-dq_max`
- Simulink Saturation-Bloecke unterstuetzen Vektoren nativ

**3. Saturation-Bloecke fuer Drehmoment** (falls vorhanden)
- Upper Limit: `tau_max` (7x1 Vektor)
- Lower Limit: `-tau_max`

**4. Gelenkgrenzen** (falls dein Modell diese nutzt)
- Ersetze `qLim_abs` (Skalar) durch `qLim_lower` und `qLim_upper` (7x1 Vektoren)

**5. URDF-Referenz**
- Stelle sicher, dass das Robot-Subsystem `SpaceKinova.urdf` nutzt

**6. Modell speichern**
- Speichere als `SpaceKinova.slx` (der Name muss zu `cfg.mdl = "SpaceKinova"` passen)

### Warum 40 Hz?
Der Kinova Gen3 nimmt im High-Level-Modus Geschwindigkeitskommandos mit **40 Hz** entgegen. Wenn dein Agent in der Simulation ebenfalls mit 40 Hz arbeitet, passen Training und echte Hardware exakt zusammen — das ist entscheidend fuer Sim-to-Real.

---

## Phase 3: PPO-Agent trainieren

### Was passiert hier?
`SpaceKinovaDynamic_PPO.m` ist das Herzstück des Projekts. Es:
1. Erzeugt eine Kreis-Referenztrajektorie
2. Berechnet per Inverse Kinematik die gewuenschten Gelenkwinkel
3. Baut die RL-Umgebung (Observation/Action Spaces)
4. Trainiert einen PPO-Agenten ueber viele Episoden

Der Agent lernt, welche **Gelenkgeschwindigkeiten** (`dq_cmd`) er zu jedem Zeitpunkt kommandieren muss, damit der Endeffector der Kreisbahn folgt.

### So startest du das Training

```matlab
SpaceKinovaDynamic_PPO
```

### Was im Detail passiert

#### Schritt 1: Konfiguration laden
Das Skript setzt alle Kinova Gen3 Spezifikationen:

```
Gelenkgrenzen:
  J1, J3, J7 (continuous): +/- 2*pi rad (Software-Limit)
  J2: +/- 2.41 rad
  J4: +/- 2.66 rad
  J5: +/- 2.23 rad
  J6: +/- 2.01 rad

Geschwindigkeitslimits (aus der Kinova URDF):
  J1-J4 (grosse Aktuatoren): 1.3963 rad/s
  J5-J7 (kleine Aktuatoren): 1.2218 rad/s

Action-Limits (70% Sicherheitsfaktor):
  J1-J4: +/- 0.977 rad/s
  J5-J7: +/- 0.855 rad/s
```

#### Schritt 2: Referenztrajektorie
Eine Kreisbahn in der x-y-Ebene:
- Mittelpunkt: [4.1, 0, 0] m
- Radius: 0.4 m
- Dauer: 8.5 s
- Die Geschwindigkeitsreferenz wird durch numerische Differentiation berechnet

#### Schritt 3: Inverse Kinematik
Fuer jeden Punkt auf der Kreisbahn wird die IK geloest, um die gewuenschten Gelenkwinkel `q_des` zu bekommen. Diese dienen als Startpositionen fuer die Episoden.

#### Schritt 4: Observation Space (29 Dimensionen)
Was der Agent in jedem Zeitschritt "sieht":

```
Observation = [
    ep      (3D)  - Positionsfehler: Wo soll der EE hin vs. wo ist er?
    ev      (3D)  - Geschwindigkeitsfehler: Wie schnell soll er vs. wie schnell ist er?
    q       (7D)  - Aktuelle Gelenkwinkel
    dq      (7D)  - Aktuelle Gelenkgeschwindigkeiten
    v_base  (3D)  - Translationsgeschwindigkeit der Basis
    w_base  (3D)  - Rotationsgeschwindigkeit der Basis
    e_ori   (3D)  - Orientierungsfehler des Endeffektors
]
```

> **Wichtig fuer Sim-to-Real:** Im Training bewegt sich die Basis (freischwebend). Auf dem echten Kinova (fest montiert) sind `v_base` und `w_base` immer `[0,0,0]`. Damit der Agent damit klarkommt, werden 30% der Trainingsepisoden mit fixierter Basis trainiert (Domain Randomization).

#### Schritt 5: Action Space (7 Dimensionen)
Was der Agent ausgibt: **7 Gelenkgeschwindigkeiten** `dq_cmd`, jeweils begrenzt auf 70% der echten Kinova-Limits.

#### Schritt 6: PPO-Training
- **ExperienceHorizon:** 512 Schritte (~12.8 s bei 40 Hz)
- **MiniBatchSize:** 128
- **Epochen pro Update:** 10
- **DiscountFactor:** 0.99
- **Entropy:** 0.01 (foerdert Exploration)
- **Max Episoden:** 1000

#### Schritt 7: Agent speichern
Der beste Agent wird automatisch in `savedAgents_spacekinova/` gespeichert. Am Ende wird der finale Agent zusaetzlich mit Konfiguration und Trainingsstatistiken gespeichert.

### Training ueberwachen
Waehrend des Trainings siehst du ein Live-Diagramm:
- **x-Achse:** Episode
- **y-Achse:** Reward
- Der Reward sollte ueber die ersten 200-500 Episoden ansteigen
- Wenn der Reward konstant bleibt oder faellt: Simulink-Modell pruefen

### Typische Trainingsdauer
- ~1-3 Stunden je nach Hardware (1000 Episoden)
- Auf einer GPU-Workstation deutlich schneller

### Was du am Ende hast
Eine `.mat` Datei in `savedAgents_spacekinova/`, z.B.:
```
savedAgents_spacekinova/ppo_spacekinova_vel_20240615_143022.mat
```
Diese enthaelt:
- `agent` — der trainierte PPO-Agent
- `cfg` — die Trainingskonfiguration
- `trainingStats` — Reward-Verlauf etc.

---

## Phase 4: Hardware-Test (Kinova via ROS)

### Was passiert hier?
Bevor du den trainierten Agent auf den echten Roboter loslässt, testest du mit `kinova_test.m`, ob die ROS-Verbindung funktioniert und der Roboter auf Geschwindigkeitskommandos reagiert. Das Skript bewegt jeden Joint einzeln mit sehr kleinen Geschwindigkeiten.

### Voraussetzungen
1. Kinova Gen3 eingeschaltet und im Netzwerk
2. `kortex_driver` laeuft:
   ```bash
   roslaunch kortex_driver kortex_driver.launch
   ```
3. Pruefen, ob Topics verfuegbar sind:
   ```bash
   rostopic list | grep my_gen3
   # Sollte u.a. zeigen:
   # /my_gen3/in/joint_velocity
   # /joint_states
   ```

### So gehst du vor

```matlab
kinova_test
```

### Was das Skript macht

1. **ROS-Verbindung herstellen:** Verbindet sich mit dem ROS Master (Standard: `192.168.1.10:11311`)
2. **Subscriber/Publisher erstellen:**
   - Subscriber: `/joint_states` (liest aktuelle Gelenkwinkel/-geschwindigkeiten)
   - Publisher: `/my_gen3/in/joint_velocity` (sendet Geschwindigkeitskommandos)
3. **Pro Joint testen:**
   - Wartet auf ENTER (du bestaetigst jeden Joint einzeln)
   - Sendet 0.1 rad/s fuer 1.5 Sekunden (positive Richtung)
   - Stoppt fuer 1.0 Sekunde
   - Sendet -0.1 rad/s fuer 1.5 Sekunden (negative Richtung)
   - Stoppt
4. **Sicherheit:** Bei CTRL+C oder Fehler wird sofort Zero-Velocity gesendet

### Sicherheitsmechanismen
- **Per-Joint Saettigung:** Nur 20% der echten Limits (max ~0.28 rad/s)
- **Watchdog:** Wenn 100 ms kein `joint_state` kommt -> Fehler (Roboter stoppt)
- **onCleanup:** Bei CTRL+C wird automatisch ein Stop-Kommando gesendet
- **Manuelle Bestaetigung:** Du drueckst ENTER vor jedem Joint-Test

### Was du pruefen solltest
- [ ] Jeder Joint bewegt sich in **beide** Richtungen
- [ ] Die Bewegung ist **gleichmaessig** (kein Ruckeln)
- [ ] Die Geschwindigkeit ist **sehr langsam** (~6 Grad/Sekunde)
- [ ] Der Roboter **stoppt zuverlaessig** nach jedem Test

### Konfiguration anpassen (falls noetig)

```matlab
% In kinova_test.m, Zeilen 14-34:
cfg.rosMasterURI = "http://192.168.1.10:11311";  % <- IP deines ROS Masters anpassen!
cfg.ns           = "/my_gen3";                     % <- Namespace pruefen
```

---

## Phase 5: Trainierten Agent auf Kinova deployen

### Was passiert hier?
`deploy_agent_kinova.m` ist die Bruecke zwischen Simulation und Realitaet. Es laedt den trainierten Agent, verbindet sich mit dem Kinova, liest in Echtzeit die Gelenkzustaende, berechnet die Observation (identisch zum Training), fragt den Agent nach dem naechsten Geschwindigkeitskommando und sendet es an den Roboter — 40 Mal pro Sekunde.

### Ablauf: Erst Dry-Run, dann echte Ausfuehrung

#### Schritt 1: Dry-Run (PFLICHT!)

```matlab
% deploy_agent_kinova.m oeffnen
% cfg.dryRun = true;  (ist bereits Standard)

deploy_agent_kinova
```

Im Dry-Run Modus:
- **Keine** ROS-Verbindung wird hergestellt
- Kommandos werden nur auf der Konsole **angezeigt**, nicht gesendet
- Du siehst einmal pro Sekunde eine Zeile wie:
  ```
  t=1.00s | ep=[0.012 -0.003 0.001]m | dq_cmd=[0.1234 -0.0567 0.0891 ...] rad/s
  ```

**Pruefen:** Sind die Geschwindigkeiten plausibel? Alle unter 1 rad/s? Keine NaN/Inf Werte?

#### Schritt 2: Echte Ausfuehrung

```matlab
% In deploy_agent_kinova.m:
cfg.dryRun = false;   % <- ERST nach erfolgreichen Dry-Run aendern!

% Optional: Kurze Laufzeit fuer Ersttest
cfg.maxDuration = 3.0;   % Nur 3 Sekunden statt 8.5

deploy_agent_kinova
```

Das Skript zeigt Sicherheitshinweise und wartet auf ENTER. Dann:

1. **Zustand lesen:** Aktuelle Gelenkwinkel/-geschwindigkeiten vom Kinova
2. **Forward Kinematics:** Berechnet wo der Endeffector gerade ist
3. **Observation berechnen:** Positionsfehler, Geschwindigkeitsfehler, etc. (29D)
4. **Agent abfragen:** `getAction(agent, obs)` -> 7 Geschwindigkeitskommandos
5. **Sicherheitsfilter:**
   - Per-Joint Saettigung auf 70% der Limits
   - Soft-Limit Bremsung: 10 Grad vor Gelenklimit wird langsamer
   - OOD-Check: Wenn Positionsfehler > 0.4 m -> Sofort-Stopp
6. **Kommando senden:** ueber ROS an `/my_gen3/in/joint_velocity`

### Sicherheitskonzept im Detail

```
                    Agent Output
                         |
                    [dq_cmd raw]
                         |
              +----------v-----------+
              | Per-Joint Saettigung |  <- max 70% der Hardware-Limits
              +----------+-----------+
                         |
              +----------v-----------+
              | Soft-Limit Bremsung  |  <- Geschwindigkeit runterfahren
              | (10° vor Gelenklimit)|     nahe am Gelenklimit
              +----------+-----------+
                         |
              +----------v-----------+
              | OOD-Erkennung        |  <- Stopp wenn Positionsfehler
              | (ep > 0.4m -> Stop)  |     zu gross (Agent ueberfordert)
              +----------+-----------+
                         |
              +----------v-----------+
              | Watchdog             |  <- Stopp wenn kein joint_state
              | (100ms Timeout)      |     empfangen wird
              +----------+-----------+
                         |
                  [dq_cmd sicher]
                         |
                  An Kinova senden
```

### Wie der Agent den Kinova ansteuert (technisch)

Der Agent gibt pro Zeitschritt 7 Geschwindigkeitswerte aus — einen pro Gelenk:

```
dq_cmd = [dq1, dq2, dq3, dq4, dq5, dq6, dq7]   (in rad/s)
```

Diese werden als `kortex_driver/Base_JointSpeeds` Nachricht verpackt:
- Jedes Gelenk bekommt seinen `JointIdentifier` (0-6, **0-basiert**)
- Jedes Gelenk bekommt seinen `Value` (Geschwindigkeit in rad/s)
- `Duration = 0` (der Roboter fuehrt den Befehl bis zum naechsten aus)

Der Kinova empfaengt diese Nachricht auf dem Topic `/my_gen3/in/joint_velocity` und bewegt die Gelenke entsprechend.

### Unterschied Training vs. echte Hardware

| Aspekt | Training (Simulation) | Deployment (Hardware) |
|--------|----------------------|----------------------|
| Basis | Frei schwebend (6 DOF) | **Fest montiert** |
| v_base, w_base | Variabel (bewegt sich) | **Immer [0,0,0]** |
| Physik | Simscape (idealisiert) | Echte Physik + Reibung |
| Kontrollrate | 40 Hz (simuliert) | 40 Hz (Echtzeit) |
| Saettigung | 70% der Limits | 70% der Limits (identisch) |

> **Warum funktioniert das trotzdem?**
> - 30% der Trainingsepisoden nutzen `v_base = w_base = [0,0,0]` (Domain Randomization)
> - Die Wuerfelbasis hat 65 kg und bewegt sich daher im Training sowieso nur minimal
> - Der Agent hat also gelernt, auch mit nahezu statischer Basis zu arbeiten

### Was du am Ende bekommst

1. **Log-Datei** (`deploy_log_<timestamp>.mat`):
   - Zeitverlauf aller Gelenkwinkel, -geschwindigkeiten und Kommandos
   - Endeffector-Position vs. Referenz
   - Positionsfehler

2. **Plots:**
   - End-Effector Tracking: Soll vs. Ist Position (x, y, z)
   - Positionsfehler-Norm ueber die Zeit
   - Kommandierte Gelenkgeschwindigkeiten ueber die Zeit

3. **Konsolen-Ausgabe:**
   ```
   Position Tracking RMSE: 0.0234 m
   Position Tracking Max:  0.0512 m
   ```

---

## Zusammenfassung: Checkliste

### Einmalige Vorbereitung
- [ ] MATLAB mit allen Toolboxen installiert
- [ ] `ros_kortex` URDF fuer Kinova Gen3 7-DOF beschafft
- [ ] `kortex_driver` auf ROS-Rechner installiert und getestet
- [ ] Netzwerk zwischen MATLAB-Rechner und Kinova konfiguriert

### Workflow pro Experiment

```
1. [ ] make_spacekinova_urdf("kinova_gen3.urdf", "SpaceKinova.urdf")
        -> Ergebnis: SpaceKinova.urdf mit 7 DOF

2. [ ] Simulink-Modell pruefen/anpassen
        -> RL_Agent Sample Time = 0.025
        -> Saturations nutzen 7x1 Vektoren

3. [ ] SpaceKinovaDynamic_PPO ausfuehren
        -> Training laeuft, Reward-Kurve beobachten
        -> Agent wird in savedAgents_spacekinova/ gespeichert

4. [ ] kinova_test ausfuehren
        -> Jeden Joint einzeln testen
        -> ROS-Verbindung validieren

5. [ ] deploy_agent_kinova mit cfg.dryRun = true
        -> Kommandos pruefen (plausibel? innerhalb Limits?)

6. [ ] deploy_agent_kinova mit cfg.dryRun = false
        -> E-Stop bereit, Arbeitsraum frei
        -> cfg.maxDuration = 3.0 fuer Ersttest
        -> Schrittweise Dauer erhoehen
```

### Sicherheitsregeln
1. **Nie** ohne vorherigen Dry-Run deployen
2. **Nie** den Sicherheitsfaktor ueber 0.8 setzen
3. **Immer** Not-Aus griffbereit haben
4. **Immer** zuerst `kinova_test.m` erfolgreich durchfuehren
5. **Immer** mit kurzer Laufzeit (3s) anfangen und schrittweise erhoehen

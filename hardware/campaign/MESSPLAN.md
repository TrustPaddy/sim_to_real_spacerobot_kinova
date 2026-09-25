# Messplan Hardware-Kampagne 2026

Ziel: Die Hardware-Ergebnisse des Papers (Table III, Table IV, Fig. 7) auf eine kontrollierte, vollständig
geloggte Messreihe stellen. Die alten Läufe mischen Skriptstände, Bahndauern, Befehlsfaktoren und einen Fehler im
Beobachtungsvektor (Befunde A17 bis A22, A37 bis A40 in `Reviews/Submission_1/reviewer_comments.md` des
Paper-Repos). Die neue Reihe trennt diese Einflüsse.

Alle Bedingungen stehen in `campaign_plan.m`. Diese Datei beschreibt Ablauf, Begründung und Prüfungen.

## Fragen, die die Reihe beantworten soll

1. Läuft der 40-Hz-Agent mit korrigierter Beobachtung, Wandzeit-Referenz (ohne Drossel) und ungeskalierten
   Befehlen auf der Trainingsbahn (8,5 s) durch? (`T40_cdr_nom`, optional `T40_ppo_nom`)
2. Läuft der 10-Hz-Agent unter genau denselben Bedingungen durch, und wie genau folgt er? (`T10_nom`)
3. Wie verteilt sich die Loop-Zeit auf Senden, Feedback, FK, getAction, Kette und Logging? (Timing `M_*` und die
   Teilzeiten jedes Tracking-Laufs)
4. Verhält sich V2.4 wie V2.3? (`R0_v23_repro`, Vergleich mit den Läufen 074 bis 078)
5. Wie hängt der Erfolg des Set-Point-Agenten vom Startabstand $d_0$ ab, bei festem Ziel und festen Startposen?
   (`S10_nom`, Reproduktion `S0_repro073`, siehe Abschnitt „Set-Point-Reihe“)

Frage 1 entscheidet, ob die Ratendiagnose der Hauptbeitrag des Papers bleibt.

## Aufbau (für alle Läufe gleich)

| Punkt | Festlegung |
|---|---|
| Rechner | Deploy-Laptop HP Pavilion x360 (Intel Core i5-1135G7, 16 GB), Netzbetrieb, Energiesparplan „Höchstleistung“, keine anderen Programme offen |
| Verbindung | Ethernet direkt zum Gen3, IP 192.168.0.10 |
| MATLAB | R2026a seit 25.09.2026 (Version steht im Log), `setup_project` ausführen. Die alten Läufe 008–088 liefen mit R2025b, Loop-Zeiten deshalb nur innerhalb der neuen Reihe vergleichen |
| Code | Stand vorher committen. Der Git-Hash steht in jedem Log, `gitDirty` muss 0 sein |
| Startpose | Trainingspose (Nullstellung). Das Skript fährt sie per `ReachJointAngles` an und bricht ab, wenn der Endeffektor mehr als 2 cm von der Referenz entfernt ist |
| Referenz | Halbkreis, r = 0,2 m, x-z-Ebene, Start [0, −0,025, 1,687] m, Wandzeit (`referenceTiming = 'wall'`) |
| Sicherheit | HW-Cap 75 % der Gelenkgrenzen, Soft-Limit-Bremsung 10°, OOD-Stopp bei 0,4 m, Watchdog max(0,1 s; 1,5 × Taktperiode), E-Stop in Reichweite, zweite Person im Labor |
| Befehlsfaktor | 1,0. Nur wenn das aus Sicherheitsgründen nicht geht: 0,5 für **alle** Bedingungen (`T10_s05`, `T40_cdr_s05`) |

## Ablauf eines Labortags

1. Laptop starten, MATLAB öffnen, `setup_project`, Code-Stand prüfen (`git status` sauber).
2. Trockenlauf zur Kontrolle: `deploy_tracking_v24('T10_nom', 0, 'dryRun', true)`. Er schreibt nach
   `data/hardware/campaign/_dryrun/` und bewegt nichts.
3. Timing-Messungen in der Reihenfolge von `campaign_plan().order` (Roboter steht, es wird nur Null gesendet):
   `measure_loop_timing('M_send', 1)`, dann `M_fb`, `M_fk`, `M_full`, jeweils Wiederholung 1 bis 3.
4. Reproduktion: `deploy_tracking_v24('R0_v23_repro', 1)` und `(…, 2)`. Erwartung wie 074 bis 078: durchgelaufen,
   RMS etwa 0,05 m. Weicht das deutlich ab, erst die Ursache klären und keine weiteren Läufe fahren.
5. Hauptreihe abwechselnd: `T10_nom` r1, `T40_cdr_nom` r1, `T10_nom` r2, `T40_cdr_nom` r2, … bis r5.
6. Wenn Zeit bleibt: `T40_ppo_nom` r1 bis r5.
7. Nach jedem Lauf die Konsolenausgabe prüfen (Stoppgrund, RMS, wirksamer Faktor). Auffälligkeiten im Laborbuch
   festhalten. Was vorher bekannt ist, per `'operatorNote', '...'` beim Aufruf mitgeben. Dateien nie umbenennen.
8. Am Ende: `python evaluation/campaign/analyze_campaign.py` und die Ergebnisse in `data/hardware/campaign/results/`
   ansehen. Logs und Ergebnisse committen.

Ein abgebrochener Lauf (OOD, Watchdog, Fault) ist ein gültiges Ergebnis und wird nicht wiederholt. Wird ein Lauf aus
einem anderen Grund verworfen (zum Beispiel Kabel gestört, Person im Arbeitsraum), bleibt die Datei liegen und der
Grund kommt ins Laborbuch. Die Wiederholung bekommt die nächste freie Nummer.

## Was jeder Log enthält

`deploy_tracking_v24` speichert `data/hardware/campaign/<condId>/<condId>_rNN_<Zeit>.mat` mit der Variable `run`:

- `run.meta`: Bedingung, Wiederholung, Skriptversion, Git-Hash, Rechner, CPU, MATLAB-Version, Agent-Datei mit MD5
  und Abtastzeit, Befehlsfaktor, Referenzmodus, Startpose und Startabweichung, Stoppgrund und Stoppschritt,
  Kennzahlen (RMS, Max, RMS der ersten 3,4 s, Loop-Zeiten, gemessener Faktor).
- `run.cfg`: alle Einstellungen.
- `run.log`: pro Schritt Wandzeit, Referenzzeit, alle Teilzeiten (`dur_feedback`, `dur_fk`, `dur_obs`, `dur_agent`,
  `dur_pipeline`, `dur_send`, `dur_log`, `dur_wait`), Gelenkzustand, FK- und Kortex-Pose, Referenz, Fehler,
  Observation vor und nach dem Clipping, alle Stufen der Befehlskette bis zum gesendeten Wert in deg/s,
  Flags für Soft-Limit und HW-Cap, Schrittart (0 normal, 1 OOD-Stopp, 2 anderer Stopp).

`measure_loop_timing` speichert entsprechend nach `data/hardware/campaign/<M_...>/`.

`deploy_setpoint_v24` speichert `data/hardware/campaign/<condId>/<condId>_<Start>_rNN_<Zeit>.mat` (Schema
`sk_campaign_setpoint_v1`) mit denselben Feldern wie oben und zusätzlich: FK-Endeffektor, aus der FK
vorhergesagte und gemessene `tool_pose`, aus der `tool_pose` zurückgerechneter Endeffektor-Punkt, kleinste Höhe
über der Montagefläche, Konvergenz-Flag und Haltezeit. Schrittart 3 kennzeichnet den Konvergenz-Stopp.

## Auswertung

`evaluation/campaign/analyze_campaign.py` liest nur diese Logs und rechnet alles neu:

| Ausgabe | Verwendung |
|---|---|
| `table3_looprate.tex`, `table3_breakdown.csv` | Table III und die Aufschlüsselung der Loop-Zeit in Sec. V-C |
| `table4_tracking.tex` | Table IV |
| `fig7_loop_histogram.pdf` | Fig. 7 |
| `table_setpoint_hw.tex`, `fig_setpoint_d0.pdf` | Set-Point-Tabelle (ersetzt Table VIII) und Endfehler über $d_0$ |
| `runs_tracking.csv`, `runs_timing.csv`, `runs_setpoint.csv` | Übersicht aller Läufe |
| `paper_numbers.csv` | jede Tabellenzahl mit den Dateien, aus denen sie stammt |

## Set-Point-Reihe (R1.9)

Ziel: Erfolg über dem Startabstand $d_0$ bei festem Ziel zeigen. Die alten sechs Läufe 068 bis 073 hatten
beliebige Start- und Zielpunkte, zwei Faktoren (0,5 und 0,75) und einen anderen Regelpunkt als im Training
(Befund A42). Die Reihe kann am selben Tag wie die Tracking-Reihe laufen oder an einem eigenen Tag.

| Punkt | Festlegung |
|---|---|
| Agent | `SavedAgents/MotionProfile/point/test_agent_fixed1.mat` (Fixed-Base-Training `SpaceKinova_Point_CDR.m`), 10 Hz |
| Ziel | Nominales Trainingsziel [0,479, −0,005, 1,136] m im URDF-Frame, im Kortex-Frame [0,479, −0,005, 0,636] m (Endeffektor-Punkt ohne Greifer) |
| Startposen | S00 (Trainingsanker) und S01 bis S14, je zwei pro Stufe $d_0$ = 0,10 / 0,20 / 0,35 / 0,50 / 0,70 / 0,90 / 1,10 m. Definiert in `setpoint_starts.mat` und `.csv`, Übersicht in `setpoint_starts_map.png` und `setpoint_starts_overview.png` |
| Regelpunkt | FK der URDF wie im Training. Die Kortex-`tool_pose` wird nur geloggt |
| Befehlsfaktor | 1,0 (`S10_nom`). Nur wenn das nicht geht: 0,5 für **alle** Läufe (`S10_s05`) |
| Dauer | höchstens 25 s, wie eine Trainingsepisode |
| Abbruch | Konvergenz wie der Erfolgsabbruch im Training ($\lVert e_p\rVert$ < 2 cm und $\lVert e_v\rVert$ < 3 cm/s, 0,5 s gehalten), OOD bei $\lVert e_p\rVert$ > min(2 m; $d_0$ + 0,4 m), Höhenwächter 5 cm über der Montagefläche, Watchdog 0,15 s |
| Wiederholungen | 2 je Start, Reihenfolge aus `campaign_plan().orderSetpoint` (Durchgang 1 nach aufsteigendem $d_0$, Durchgang 2 absteigend) |
| Erfolg | Endfehler < 50 mm wie in Table VIII. Zusätzlich wird gezählt, wie oft das Trainingskriterium erreicht wurde (Stoppgrund `converged`) |

Ablauf:

1. **Startliste ansehen.** `setpoint_starts_map.png` zeigt alle Startpunkte in Draufsicht und Seitenansicht. Die
   Liste wurde mit `make_setpoint_starts` ohne Kenntnis der Zelle erzeugt (Filter: Abstand zu den Gelenkgrenzen,
   Höhe über der Montagefläche, Selbstkollision, Singularität, Anfahrweg vom Anker). Die Liste nicht neu erzeugen,
   sonst werden alle Freigaben ungültig.
2. **Anfahrtest.** `check_setpoint_starts` fährt die Zielpose Z00 und alle Starts vom Anker aus an. Jede Pose auf
   Hindernisse prüfen (Satelliten-Mockup, zweiter Roboter, Linearachse, Kabel) und mit j oder n beantworten. Die
   Antworten landen in `data/hardware/campaign/setpoint_start_check.csv` und gelten nur für diese Version der
   Startliste (MD5). Gesperrte Posen fallen aus der Reihe und kommen ins Laborbuch. Sie werden nicht ersetzt.
   Die Ausgabe zeigt auch die Abweichung zwischen gemessener und aus der FK vorhergesagter `tool_pose`. Sie sollte
   unter etwa 1 cm liegen, sonst stimmt der Werkzeug-Offset nicht.
3. **Reproduktion.** `deploy_setpoint_v24('S0_repro073', 'S00', 1)`. Die Bedingung bildet Lauf 073 nach
   (Kortex-`tool_pose`, Faktor 0,5, 60 s, Ziel wie 073). Erwartung wie 073: Konvergenz nach etwa 22 s, Endfehler
   etwa 17 mm. Der kinematische Trockenlauf ergab 22,6 s und 19,4 mm.
4. **Hauptreihe.** In der Reihenfolge von `campaign_plan().orderSetpoint`, zum Beispiel
   `deploy_setpoint_v24('S10_nom', 'S02', 1)`. Das Skript fährt vor jedem Lauf über den Anker in die Startpose und
   fragt vor der Anfahrt und vor dem Lauf nach ENTER. Abgebrochene Läufe sind gültige Ergebnisse.
5. **Auswertung** wie unten. Zusätzlich entstehen `runs_setpoint.csv`, `table_setpoint_hw.tex` und
   `fig_setpoint_d0.pdf`. `runs_setpoint.csv` zeigt je Lauf, wie lange die Soft-Limit-Bremsung aktiv war und
   welche Gelenke an ihre Hardwaregrenze kamen.

Ein Trockenlauf (`'dryRun', true`) integriert die Gelenke kinematisch wie das Trainingsmodell, ohne Verzögerung
und Rauschen. Er zeigt vor dem Labor, ob der Agent von einer Startpose aus konvergiert. Ergebnis vom 22.09.2026
für alle 15 Starts: Endfehler 6 bis 23 mm, 13 Starts konvergiert, S00 und S04 bleiben nach 25 s bei etwa 22 mm
stehen. Bei beiden steht J6 an seiner Grenze von 115,2° und die Soft-Limit-Bremsung ist aktiv (Befund A43).

### Befunde beim Vorbereiten

- **Werkzeug-Offset (A42).** Die Kortex-`tool_pose` liegt 0,121 m entlang der Werkzeug-z-Achse vor dem
  Endeffektor-Punkt der URDF, weil im Kortex-Tool der Greifer eingetragen ist. Der URDF-Frame liegt außerdem 0,5 m
  unter dem Kortex-Frame (`base_to_kinova`). Fit über 271 Punkte der Läufe 068 bis 073, Restfehler RMS 3 mm,
  maximal 10 mm. V3.0 hat die `tool_pose` als Endeffektor genutzt. Die alten Set-Point-Läufe haben damit einen um
  12 cm versetzten Punkt geregelt, und ihre Ziele und $d_0$ beziehen sich auf diesen Punkt. Das Tracking-Skript
  nutzt die FK und ist nicht betroffen.
- **Gelenkgrenzen (A43).** Im Trainingsmodell wird die Gelenkposition hinter dem Integrator hart geklemmt (J2
  ±138,1°, J4 ±152,4°, J6 ±115,2°), ohne Anti-Windup. Der Gen3 hat J2 ±128,9°, J4 ±147,8° und J6 ±120,3°. Das
  Deploy-Skript bremst 10° vor den Trainingsgrenzen ab. In den Läufen 069 und 070 (nicht konvergiert, volle 60 s)
  stand J6 in 91 % der Schritte in der Bremszone bei 115,2°. In Lauf 072 (steckengeblieben, 773,6 mm) stand J4 an
  der Hardwaregrenze −147,8°, in Lauf 073 ebenfalls, dort trotzdem mit Erfolg. Die neue Reihe loggt dazu alles
  Nötige. Die Grenzen im Skript bleiben wie im Training, damit die Reihe mit dem Training vergleichbar bleibt.

## Noch offen

- Open-Loop-Playback (Sec. V-B) muss nicht wiederholt werden. Die Läufe 001 bis 006 reichen und sind ausgewertet.

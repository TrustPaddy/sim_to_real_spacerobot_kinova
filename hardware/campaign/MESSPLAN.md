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

Frage 1 entscheidet, ob die Ratendiagnose der Hauptbeitrag des Papers bleibt.

## Aufbau (für alle Läufe gleich)

| Punkt | Festlegung |
|---|---|
| Rechner | Deploy-Laptop HP Pavilion x360 (Intel Core i5-1135G7, 16 GB), Netzbetrieb, Energiesparplan „Höchstleistung“, keine anderen Programme offen |
| Verbindung | Ethernet direkt zum Gen3, IP 192.168.0.10 |
| MATLAB | R2025b (Version steht im Log), `setup_project` ausführen |
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

## Auswertung

`evaluation/campaign/analyze_campaign.py` liest nur diese Logs und rechnet alles neu:

| Ausgabe | Verwendung |
|---|---|
| `table3_looprate.tex`, `table3_breakdown.csv` | Table III und die Aufschlüsselung der Loop-Zeit in Sec. V-C |
| `table4_tracking.tex` | Table IV |
| `fig7_loop_histogram.pdf` | Fig. 7 |
| `runs_tracking.csv`, `runs_timing.csv` | Übersicht aller Läufe |
| `paper_numbers.csv` | jede Tabellenzahl mit den Dateien, aus denen sie stammt |

## Noch offen

- Set-Point-Reihe über festgelegte Startabstände (R1.9). Dafür braucht `deploy_agent_kinova_point.m` dieselben
  Änderungen wie V2.4 (Stoppschritt loggen, Teilzeiten, Observation, Metadaten) und eine feste Liste von Start- und
  Zielposen. Wird als nächster Schritt vorbereitet.
- Open-Loop-Playback (Sec. V-B) muss nicht wiederholt werden. Die Läufe 001 bis 006 reichen und sind ausgewertet.

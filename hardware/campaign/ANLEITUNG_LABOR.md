# Laboranleitung Messkampagne 2026 (Kurzfassung)

Ablauf eines Labortags zum Abhaken. Begründung, Bedingungen und Log-Inhalt stehen in `MESSPLAN.md`, die
Bedingungen selbst in `campaign_plan.m`. Alle Befehle laufen im MATLAB-Command-Window.

## 0. Vorbereitung

- [ ] Laptop am Netz, Energiesparplan „Höchstleistung“, alle anderen Programme geschlossen
- [ ] Ethernet direkt zum Gen3 (IP 192.168.0.10)
- [ ] Repo aktuell: `git pull`, danach muss `git status` sauber sein. Sonst warnen die Skripte
  (`gitDirty`), und die Logs lassen sich keinem Code-Stand zuordnen
- [ ] MATLAB R2025b öffnen und `setup_project` ausführen
- [ ] Not-Aus in der Hand, zweite Person im Labor, Arbeitsraum frei
- [ ] Trockenlauf ohne Roboter: `deploy_tracking_v24('T10_nom', 0, 'dryRun', true)`

Jedes Skript fragt vor jeder Bewegung nach ENTER. CTRL+C bricht ab und sendet Geschwindigkeit null.

## Teil A: Timing und Tracking

**A1. Timing-Messungen.** Der Roboter steht, es wird nur null gesendet.

```matlab
for r = 1:3
    measure_loop_timing('M_send', r); measure_loop_timing('M_fb', r);
    measure_loop_timing('M_fk', r);   measure_loop_timing('M_full', r);
end
```

**A2. Reproduktion der alten Läufe 074 bis 078.**

```matlab
deploy_tracking_v24('R0_v23_repro', 1)
deploy_tracking_v24('R0_v23_repro', 2)
```

Erwartung: Lauf kommt durch, RMS etwa 0,05 m. Weicht das deutlich ab, **hier aufhören** und erst die Ursache
klären.

**A3. Hauptreihe.** Beide Agenten immer abwechselnd, damit eine Drift beide gleich trifft.

| r | Aufruf 1 | Aufruf 2 |
|---|---|---|
| 1 | `deploy_tracking_v24('T10_nom', 1)` | `deploy_tracking_v24('T40_cdr_nom', 1)` |
| 2 | `deploy_tracking_v24('T10_nom', 2)` | `deploy_tracking_v24('T40_cdr_nom', 2)` |
| 3 | `deploy_tracking_v24('T10_nom', 3)` | `deploy_tracking_v24('T40_cdr_nom', 3)` |
| 4 | `deploy_tracking_v24('T10_nom', 4)` | `deploy_tracking_v24('T40_cdr_nom', 4)` |
| 5 | `deploy_tracking_v24('T10_nom', 5)` | `deploy_tracking_v24('T40_cdr_nom', 5)` |

**A4. Nur wenn Zeit bleibt:** `deploy_tracking_v24('T40_ppo_nom', r)` für r = 1 bis 5.

Was bei jedem Tracking-Lauf passiert:
1. Das Skript fährt den Arm selbst in die Trainingspose. Das ist die gestreckte Senkrechtstellung, oben muss
   also Platz sein.
2. Liegt der Endeffektor dann mehr als 2 cm neben dem Bahnstart, bricht das Skript ab.
3. ENTER startet den Lauf. Die Bahn ist ein Halbkreis mit 8,5 s (bei `R0_v23_repro` 17 s).
4. Nach dem Lauf stehen in der Konsole Stoppgrund, RMS, Loop-Zeit und wirksamer Faktor. Diese Werte ins
   Laborbuch übernehmen.

## Teil B: Set-Point

**B1. Posen freigeben.** Das ist nur einmal nötig, die Freigabe bleibt gespeichert.

```matlab
check_setpoint_starts
```

- Das Skript fährt die Zielpose Z00 und alle Starts S00 bis S14 jeweils vom Anker aus an.
- Pro Pose: ENTER fährt hin, `s` überspringt, `q` beendet.
- An jeder Pose prüfen, ob sie frei ist (Satelliten-Mockup, zweiter Roboter, Linearachse, Kabel). Danach mit
  `j` freigeben oder mit `n` sperren und einen kurzen Grund angeben.
- Die angezeigte Abweichung der `tool_pose` sollte unter etwa 1 cm liegen. Sonst stimmt der Werkzeug-Offset
  nicht, dann vor den Läufen klären.
- Gesperrte Posen entfallen, sie werden nicht ersetzt. `make_setpoint_starts` **nicht** neu ausführen, sonst
  werden alle Freigaben ungültig.

**B2. Reproduktion von Lauf 073.**

```matlab
deploy_setpoint_v24('S0_repro073', 'S00', 1)
```

Erwartung: Konvergenz nach etwa 22 s, Endfehler etwa 17 mm.

**B3. Hauptreihe.** Aufruf `deploy_setpoint_v24('S10_nom', '<Start>', r)`, zum Beispiel
`deploy_setpoint_v24('S10_nom', 'S02', 1)`. Gesperrte Posen überspringen.

| Durchgang | Reihenfolge der Starts |
|---|---|
| r = 1 (nach $d_0$ aufsteigend) | S02, S01, S03, S00, S04, S06, S05, S08, S07, S10, S09, S11, S12, S14, S13 |
| r = 2 (nach $d_0$ absteigend) | S13, S14, S12, S11, S09, S10, S07, S08, S05, S06, S04, S00, S03, S01, S02 |

Die Reihenfolge kommt aus `p = campaign_plan(); p.orderSetpoint`.

Was bei jedem Set-Point-Lauf passiert:
1. ENTER fährt über den Anker in die Startpose.
2. Ein zweites ENTER startet den Lauf. Er dauert höchstens 25 s und endet früher, wenn der Agent 0,5 s lang
   im Ziel bleibt.
3. Bei S00 und S04 ist ein Stehenbleiben bei etwa 22 mm zu erwarten (J6 an der Grenze, Befund A43). Das ist
   kein Fehler.

## Regeln während der Messung

- Ein Abbruch durch OOD, Watchdog, Fault oder Höhenwächter ist ein gültiges Ergebnis und wird **nicht**
  wiederholt.
- Nur bei einer äußeren Störung wird ein Lauf verworfen, etwa bei einem gestörten Kabel oder einer Person im
  Arbeitsraum. Die Datei bleibt dann liegen, der Grund kommt ins Laborbuch, und die Wiederholung bekommt die
  nächste freie Nummer (zum Beispiel r = 6).
- Log-Dateien nie umbenennen oder löschen.
- Was vorher bekannt ist, beim Aufruf mitgeben: `deploy_tracking_v24('T10_nom', 3, 'operatorNote', '...')`.
- Falls Faktor 1,0 aus Sicherheitsgründen nicht geht, für **alle** Läufe auf die Rückfall-Bedingungen
  wechseln. Beim Tracking sind das `T10_s05` und `T40_cdr_s05`, beim Set-Point `S10_s05`. Beide Faktoren nie
  mischen.

## Nach der Messung

- [ ] Auswertung im Repo-Wurzelordner: `python evaluation/campaign/analyze_campaign.py`. Die Ergebnisse
  landen in `data/hardware/campaign/results/`. Das geht auch später am Desktop nach dem Push.
- [ ] Laborbuch-Notizen sichern
- [ ] Logs, `campaign_index.csv`, `setpoint_start_check.csv` und die Ergebnisse committen und pushen

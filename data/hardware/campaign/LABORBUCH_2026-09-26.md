# Laborbuch Messkampagne, 26.09.2026

## Umgebung

- Laptop mit Pop!_OS (Linux), MATLAB R2026a Update 4. Die alten Läufe liefen unter Windows mit R2025b.
- Kortex-Anbindung über Kinovas `matlab_simplified_api_2.2.1` (`kortexApiMexInterface.mexa64`).
- Ethernet über einen USB-LAN-Adapter, Laptop 192.168.0.100, Roboter 192.168.0.10.

## Zwei Roboter

An diesem Tag wurden zwei verschiedene Kinova Gen3 (7 DoF) verwendet:

| Teil | Roboter | Greifer | Kortex `tool_pose`-Offset entlang Werkzeug-z |
|---|---|---|---|
| A (A1–A4) | Roboter 1 | **größerer Greifer**, nicht der aus 068–078 | ≈ 0,205 m (Fit über 16 Posen, Rest RMS 4,7 mm) |
| B (ab B1, zweite Sitzung) | Roboter 2 | derselbe Greifer wie in den Läufen 068–078 | 0,121 m wie in `kortex_frames.m` (Anker: ≈ 7 mm Abweichung) |

- Aufgefallen bei B1 auf Roboter 1: Alle 16 Posen wichen zwischen Kortex-`tool_pose` und FK-Vorhersage um 81–87 mm
  ab, fast nur entlang der Werkzeugachse (Mittel 83,6 mm, Streuung 1,4 mm). Ursache: anderer, längerer Greifer.
- Deshalb wurde für Teil B auf Roboter 2 gewechselt.
- **Teil A wird nicht wiederholt.** Das Tracking nutzt die URDF-FK, die Läufe sind gültig. Beim Vergleich mit
  074–078 ist zu beachten, dass Teil A mit einem anderen Roboter und einem schwereren Greifer lief.
- **Die B1-Sitzung von 14:32 (Roboter 1) ist ungültig.** Die Freigaben stehen weiter in
  `setpoint_start_check.csv`, und `deploy_setpoint_v24` akzeptiert jede frühere Freigabe unabhängig von der Sitzung.
  Maßgeblich sind nur die Freigaben aus der B1-Sitzung auf Roboter 2. Posen, die dort gesperrt werden, in B3 von
  Hand auslassen.

## Teil A (Roboter 1)

- **A1 Timing**, 3 × 4 Messungen, alle 300 Zyklen, keine Faults:
  M_send 25,0 ms (40 Hz), M_fb / M_fk / M_full je 50,0 ms (20 Hz), p95 ≤ 54,1 ms.
  Jeder Kortex-Aufruf (`SendJointSpeedCommand`, `RefreshFeedback`) braucht ≈ 25 ms, das Netz nur 0,5 ms (Ping).
  Mit Senden und Feedback pro Zyklus sind 40 Hz über die High-Level-API nicht erreichbar.
- **A2 `R0_v23_repro`**, r1–r2: completed, RMS 0,058 / 0,059 m (alt 074–078: 0,053 m).
  Loop genau 10 Hz, die alten Windows-Läufe 074–078 hatten real ≈ 177 ms (≈ 5,6 Hz).
- **A3** `T10_nom` und `T40_cdr_nom`, je r1–r5, alle completed:
  T10 RMS 0,052–0,055 m, T40_cdr RMS 0,053–0,054 m. T40_cdr lief effektiv mit 20 Hz (50 ms).
  Reihenfolge: T10 r5 wurde nach T40_cdr r5 gefahren (anfangs übersprungen).
- **A4 `T40_ppo_nom`**, r1–r5, alle completed: RMS 0,024 m, Max 0,042–0,046 m.
  Befund: T10 und T40_cdr sind über die ersten drei Viertel der Bahn genauer als der Basis-PPO, laufen im letzten
  Viertel aber weg (mittlerer Fehler ≈ 0,10 m, Max ≈ 0,16 m am Bahnende). Der Basis-PPO bleibt bis zum Ende
  bei ≈ 0,03 m. Die Ursache ist noch offen.

## Teil B (Roboter 2)

- **B1 `check_setpoint_starts`**, Sitzung 15:03: Der erste Versuch brach vor der Anfahrt zum Anker mit
  `ReachJointAngles` errorCode 22 (`ROBOT_IN_FAULT`) ab, der Arm bewegte sich nicht. Nach Beheben des Faults lief
  der zweite Versuch durch.
  Alle 16 Posen (Z00, S00–S14) sind angefahren und freigegeben. `tool_pose`-Abweichung 1,2–11,7 mm, meist 5–7 mm.
  Die beiden größten Werte haben S09 (11,7 mm) und S11 (11,0 mm), beide knapp über 1 cm und damit im erwarteten
  Rahmen (Fit 068–073: max 10 mm). Keine Pose gesperrt.
- **B2 `S0_repro073`**, S00 r1: converged nach 216 Schritten (21,6 s), Endfehler 16,7 mm, Setzzeit (50 mm)
  8,1 s, Pfadeffizienz 0,79, Loop 100,0 ms, Faktor 0,50. Erwartet waren ≈ 22 s und ≈ 17 mm, Lauf 073 ist damit
  reproduziert.
- **B3 `S10_nom`, Durchgang r1** (15:12–15:23): alle 15 Starts erreichen das 50-mm-Kriterium. 13 davon mit
  `converged` nach 3,1–12,2 s. S00 und S04 enden erwartungsgemäß mit `timeout` nach 25 s (S04: Endfehler 22,0 mm,
  J6 an der Grenze, Befund A43).
  Reihenfolge: S02, S01, S03, S00, S04 wie geplant, danach numerisch S05–S14 statt der geplanten Reihenfolge
  (S06 vor S05, S08 vor S07, S10 vor S09, S14 vor S13).
- **B3 `S10_nom`, Durchgang r2** (15:27–15:34): numerisch absteigend S14 → S00 gefahren statt streng nach d0.
  Die Nummern sind annähernd nach d0 sortiert. Abweichungen gibt es nur bei Paaren mit fast gleichem d0 und bei
  S00, das ans Ende rückt. Ergebnis: alle 15 Starts erfolgreich (50 mm), S00 und S04 wieder mit `timeout`.
- **B3 gesamt (30 Läufe):** 30/30 erfolgreich. Endfehler 4,9–22,7 mm, Median r1 14,4 mm, r2 15,0 mm.
  Der größte Unterschied zwischen r1 und r2 am selben Start beträgt 2,9 mm (S10), die Läufe sind also sehr gut
  reproduzierbar. Kleinste Höhe über der Montagefläche 0,157 m (S09), der Höhenwächter (0,05 m) griff nie.
- **`gitDirty`-Warnung ab B2:** `campaign_index.csv` ist seit Commit c1e1330 versioniert, und jeder Lauf hängt eine
  Zeile an. Deshalb gilt das Repo ab dem ersten Lauf nach dem Commit als geändert. Der Code-Stand ist unverändert
  c1e1330, die einzige Änderung ist diese Index-Datei. Das `gitDirty = true` in den B-Logs ist deshalb ein
  Fehlalarm.

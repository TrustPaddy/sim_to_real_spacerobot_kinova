# Desktop-Plan: Simulation und Auswertung ohne Labor

Stand 24.09.2026. Aufgaben, die auf dem Desktop (AMD Ryzen 7 7700, 8 Kerne, 32 GB, MATLAB R2026a) ohne Roboter
laufen. Die Nummern (R1.6, A24, ...) verweisen auf `Reviews/Submission_1/reviewer_comments.md` im Paper-Repo. Die
Laborreihe steht in `hardware/campaign/MESSPLAN.md`. Keine dieser Aufgaben erzeugt einen Agenten, der in der
Laborreihe gebraucht wird. Einzige Ausnahme ist D10, falls das Agentenpaar auf die Hardware soll.

## Grundlage der Schätzungen

- Gemessen am 24.09.: Eine 8,5-s-Episode im 40-Hz-Modell (`SpaceKinova_MotionProfile_40Hz`, Solver 5 ms) dauert
  3,6 bis 3,9 s. Die erste Episode braucht wegen des Kompilierens 26 s. Das 10-Hz- und das Set-Point-Modell
  rechnen mit 20 ms und sind schneller (nicht gemessen, geschätzt 1 bis 3 s pro Episode).
- Training: Die Bayes-Suche lief laut Paper mit 17 PPO-Läufen zu je 1000 Episoden in 12 h. Das sind etwa 40 min
  pro Lauf mit 8 parallelen Workern (`UseParallel`, async).
- „Arbeit“ ist die Zeit für Skripte, Testläufe und die Prüfung der Ergebnisse. „Rechenzeit“ ist die reine
  MATLAB-Laufzeit. Beides sind Schätzungen. Bei Problemen mit den Modellen kann sich die Arbeit verdoppeln.
- MATLAB: Auf dem Desktop läuft nur R2026a ohne Oberfläche (`matlab -batch`). R2025b ist hier unvollständig
  installiert (kein `matlab.exe`). Neue Simulationszahlen stammen damit aus R2026a. Das kommt in `paper_numbers.md`.

## Übersicht

| ID | Aufgabe | beantwortet | Arbeit | Rechenzeit | braucht den Nutzer | wann |
|---|---|---|---|---|---|---|
| D0 | Grundlage: parametrisierte Kopie des 40-Hz-Modells, Auswerteskript mit deterministischer Policy und vollem Log | Voraussetzung für D1 bis D3, D6, D7 | 2–3 h | 10 min | nein | ✅ 24.09. |
| D1 | Ratenversuch 2×2: beide Tracking-Agenten bei 40 und 10 Hz im selben Modell | R1.6, A9, A33, A44, A39 | 1–2 h | 15 min | Bedingungen (a) | ✅ 24.09. |
| D2 | Ratenreihe, gemessener Hardware-Takt, Referenz nach Wandzeit oder Schrittzahl | R2.3, R3.2, AE.1, „≈ 18 %“ | 2–3 h | 15 min | nein | ✅ 24.09. |
| D3 | Frei schwebende gegen fest montierte Basis in Simulation | AE.2, R2.2, R3.1, `tab:scope` | 2–3 h | 10 min | nein | ✅ 24.09. |
| D4 | Set-Point in Simulation über die 15 Starts der Laborliste, 65 kg | R1.9, A24, A28, A43, Table VII | 2–3 h | 10 min | Agent für Table VII (b) | ✅ 24.09. |
| D5 | Vorhandene CDR-Agenten unter den Bedingungen des Entwurfs E1 nachrechnen | CDR-Zahlen (📄), A44, R3.5 | 2–3 h | 30 min | nein | ✅ 25.09. |
| D6 | Häufigkeit von NaN/Inf und grober Divergenz, Verteilung des Schritt-Rewards | R1.1, „≈ −13“ | 1 h | 15 min | nein | ✅ 25.09. |
| D7 | Robustheit gegen Beobachtungsrauschen (optional) | R1.7 | 1–2 h | 10 min | nein | ✅ 25.09. |
| D8 | Fig. 5 neu in Zielgröße | R1.4, AE.3 | 1 h | 5 min | Agent in Fig. 5 (c) | ✅ 25.09. |
| D9 | Vorhandenen Sprungtest des Gen3 auswerten | H1, H2 (❓) | 1 h | keine | nein | ✅ 25.09. (H2) |
| D10 | Gleiches Agentenpaar 40/10 Hz trainieren (optional) | A21, R1.6 auf der Hardware | 2–3 h | 1,5 h (1 Seed), 4–5 h (3 Seeds) | Entscheidung (d) | vor dem Labor, falls auf Hardware |
| D11 | Algorithmenvergleich neu, 6 Verfahren × 3 Seeds | A8, A20, Fig. 4, Table II | 3–4 h | 20–30 h | Entscheidung (e) | nach der Ausrichtung |

Summe D0 bis D9: etwa 15 bis 22 h Arbeit und 2 h Rechenzeit. Dort ist die Rechenzeit kein Engpass. D10 und D11
sind Trainings, dort bestimmt die Rechenzeit die Dauer. D11 belegt den Desktop 1 bis 1,5 Tage mit allen Kernen,
D1 bis D9 sollten deshalb vorher oder danach laufen. Die Zeiten der Off-Policy-Verfahren in D11 sind nicht
gemessen. Ein kurzes Probetraining (je 20 Episoden PPO und SAC, etwa 15 min) würde sie absichern.

## Einzelheiten

### D0 Grundlage ✅ 24.09.2026

Dateien in `evaluation/desktop/`:

| Datei | Inhalt |
|---|---|
| `desktop_build_model.m` | erzeugt `models/SK_desktop.slx` aus `models/SpaceKinova_MotionProfile_CDR.slx` (das 40-Hz-Modell mit zusätzlichem Delay-Block). Die Originale bleiben unverändert |
| `desktop_config.m` | Einstellungen einer Episode (Agent, Raten, Basismasse, Verzögerung, Bahn, deterministisch oder stochastisch) |
| `desktop_run_episode.m` | simuliert eine Episode und berechnet die Kennzahlen wie `calculate_kpi_spacekinova.m` (K1 bis K4, K7, K8), dazu Abbruchgrund, Sättigung von J2, J4, J6 und Anteil an der Positionssättigung |
| `desktop_check_d0.m` | Reproduktionsprüfung gegen die Originalmodelle, Ergebnis in `data/simulation/desktop/D0_check_*` |

- Variablen in `SK_desktop`: `p_Ts` (Solver), `p_Ts_agent` (Agent, Beobachtung, Reward), `p_T`, `p_base_mass`
  (Trägheit skaliert mit), `p_delay_steps` (Verzögerung in Agentenschritten), `p_damp_scale`.
- Feste Basis über `base_mass = 1e9`. Die Gelenke sind bewegungsgesteuert, deshalb braucht es keinen Umbau. Die
  Basis dreht sich dann um weniger als 2·10⁻⁹ rad.
- Die Dämpfung bleibt als Variable, wirkt aber nicht auf die Bewegung (A23).
- Zusätzlich geloggt: Rohaktion `a_raw`, `a_sat`, `a_filt`, `a_rl`, Gelenkbefehl `q_cmd`, Beobachtung `obs`,
  `is_done`.
- Policy standardmäßig deterministisch (`explore = false`) wie auf der Hardware (A28).
- Der Reward läuft in `SK_desktop` im Agenten-Takt wie im 40-Hz-Modell. Das 10-Hz-Modell rechnet ihn im
  Solver-Takt (20 ms). Returns aus verschiedenen Modellen sind deshalb nicht vergleichbar, EE-Fehler schon.
- Referenz nach Wandzeit. Die Referenz nach Schrittzahl (D2) fehlt noch.

Ergebnis der Prüfung (`D0_check_20260924_232429`, deterministisch, Halbkreis 8,5 s):

| Fall | Vergleich | größte Abweichung des EE-Fehlers |
|---|---|---|
| A | 40-Hz-Modell gegen `SK_desktop` (CDR2-4, 65 kg) | 1·10⁻⁹ m |
| B | CDR-Modell (Teststand: 16,25 kg, 2 Schritte, Dämpfung ×2) gegen `SK_desktop` | 0 |
| C | 10-Hz-Modell gegen `SK_desktop` (ppo_10hz, 10 Hz, Solver 20 ms, 65 kg) | 4·10⁻⁹ m |
| D | Wiederholung von A | 0 |
| E | ppo_10hz mit Solver 5 ms statt 20 ms | 0,16 mm, MSE −0,6 % |

`SK_desktop` reproduziert damit alle drei Originalmodelle. D1 kann beide Agenten mit Solver 5 ms rechnen.
Nach der Erweiterung für D2 erneut geprüft, gleiches Ergebnis (`D0_check_20260924_232429`).

Nebenbefunde (Einzelepisoden, deterministisch, noch keine Aussage fürs Paper):

- CDR2-4 bei 40 Hz und 65 kg: EE-MSE 0,00347 m². Unter der kombinierten Störung des CDR-Teststands
  0,00460 m² (E1 nennt 0,0049 m²).
- Feste Basis (F): EE-MSE 0,00244 m², also 30 % weniger als mit frei schwebender Basis. Die Basisbewegung trägt
  in Simulation merklich zum Fehler bei (Vorgriff auf D3).
- Stochastische Policy (G, Seed 1): EE-MSE 0,00451 m², 30 % mehr als deterministisch (A28).
- ppo_10hz: EE-MSE 0,00441 m² mit 65 kg und 0,00731 m² mit 6,5 kg (H). Der Wert 0,0074 m² im Paper passt zu
  6,5 kg (A33).
- J6 ist in allen deterministischen Läufen in 100 % der Agentenschritte gesättigt. Die Rohaktion erreicht
  |a₆| ≈ 0,73, die Grenze liegt bei 0,1 rad/s. Das zeigt sich in Simulation genauso wie in Hardware-Lauf 074
  (A39).

### D1 Ratenversuch 2×2 ✅ 24.09.2026

Skript `desktop_d1_rates.m`, Ergebnis `data/simulation/desktop/D1_rates_20260924_231424.*` (367 Episoden, 16 min).

- Agenten: `CDR2-4` (40 Hz, Bayes und CDR, Hardwareläufe 012–014), `Optimized` (40 Hz, Bayes ohne CDR),
  `PPO_base` (`SpaceKinova_PPO_agent_motionprofile.mat`, 40 Hz, vor Bayes und CDR, Hardwareläufe 008–011) und
  `ppo_10hz` (10 Hz).
- Jeder Agent bei 40 und 10 Hz in `SK_desktop`, Solver 5 ms, 65 kg, Halbkreis 8,5 s, Referenz nach Wandzeit.
  Filter und Rate Limiter rechnen im Agenten-Takt wie auf der Hardware.
- Sätze: nominal (deterministisch), `stoch` (20 Seeds), `mass` (20 Ziehungen 65 kg · max(0,5; 1 + 0,65 randn) wie
  CDR-Phase 4, deterministisch), `delay` (0,1 s und 0,2 s, also 4 und 8 Schritte bei 40 Hz, 1 und 2 bei 10 Hz),
  `e1` (CDR2-4 unter der kombinierten Störung aus E1). `ppo_10hz` zusätzlich mit 6,5 kg.
- Die Verzögerung ist in Sekunden angesetzt. Gleiche Schrittzahl wäre bei 10 Hz die vierfache Zeit.

Ergebnis EE-MSE [m²] (Satz `mass`, Mittelwert ± Std über 20 Ziehungen, alle Episoden ohne Abbruch):

| Agent | trainiert | bei 40 Hz | bei 10 Hz | Änderung |
|---|---|---|---|---|
| CDR2-4 | 40 Hz | 0,00340 ± 0,00033 | 0,00398 ± 0,00031 | +17 % |
| Optimized | 40 Hz | 0,00347 ± 0,00046 | 0,00438 ± 0,00049 | +26 % |
| PPO_base | 40 Hz | 0,000126 ± 0,000039 | 0,00127 ± 0,00018 | ×10 |
| ppo_10hz | 10 Hz | 0,00423 ± 0,00042 | 0,00429 ± 0,00048 | +1 % |

- **Keine der 367 Episoden bricht ab**, auch nicht bei 10 Hz, mit 0,2 s Verzögerung oder unter der E1-Störung.
  Der größte Fehler liegt bei allen Agenten unter 0,23 m. In Simulation führt die niedrigere Rate allein also nicht
  zum Scheitern der 40-Hz-Agenten. Sie vergrößert den Fehler um 17 bis 26 %, beim PPO_base um das Zehnfache auf
  kleinem Niveau (höchstens 5,6 cm).
- Getestet sind nur ideale Bedingungen: Referenz nach Wandzeit, fester Takt ohne Jitter, keine Latenz, korrekte
  Beobachtung. Die Hardwareläufe der 40-Hz-Agenten wichen davon ab (Beobachtungsfehler A22, Skalierung A18,
  Jitter, Referenztakt). Welcher dieser Faktoren das Scheitern erklärt, zeigen D2 und die Laborreihe (`T40_cdr_nom`).
- CDR2-4, Optimized und ppo_10hz folgen der Bahn anfangs gut und fallen zum Bahnende zurück. Ihr größter Fehler
  (17–19 cm) liegt bei allen am Ende (t = 8,5 s). PPO_base folgt über die ganze Bahn auf 2 cm genau, bewegt die
  Basis dafür stärker (Orientierungsfehler bis 0,056 rad statt 0,011 bis 0,018 rad) und nutzt J2 und J4 bis zur
  Sättigung. Das passt zur Gewichtung von Tracking gegen Basisruhe im Reward.
- Verzögerung: 0,2 s erhöht das EE-MSE bei 40 Hz um 7 bis 52 % (PPO_base ×37), ohne Abbruch.
- Stochastische Policy: CDR2-4 0,00379 m² bei 40 Hz und 0,00492 m² bei 10 Hz (je 20 Seeds). Die Ratenwirkung ist
  damit stochastisch größer (+30 %) als deterministisch (+17 %).
- A44: CDR2-4 unter der E1-Störung ergibt 0,00460 m² deterministisch und 0,00503 ± 0,00030 m² stochastisch.
  E1 nennt 0,0049 m², das liegt im Bereich der stochastischen Auswertung.
- A33: ppo_10hz mit 6,5 kg ergibt 0,00728 m² (Paper 0,0074 m²), mit 65 kg 0,00438 m². Der Vergleich in Sec. VI-A
  (0,0049 gegen 0,0074 m²) vergleicht damit auch 65 kg mit 6,5 kg. Bei gleicher Masse und gleicher Auswertung
  liegen CDR2-4 (0,00347) und ppo_10hz (0,00438) bei ihrer jeweiligen Trainingsrate näher beieinander.
- A39: J6 ist bei CDR2-4 und ppo_10hz in 100 % der Agentenschritte gesättigt, bei Optimized und PPO_base in 91 bis
  98 %.

### D2 Ratenreihe, Referenztakt und Nachbau der Hardwareläufe ✅ 24.09.2026

Skript `desktop_d2_timing.m` (Teile A bis E), Ergebnisse `data/simulation/desktop/D2_timing_20260924_232707`
(B), `_233214` (A, C), `_233400` (D), `_233557` (E). Dafür hat `SK_desktop` drei Schalter bekommen:
Befehlsskalierung (`p_cmd_scale`), Anstieg des Rate Limiters (`p_slew`) und einen Block „Obs Transform“ vor dem
Agenten, der wahlweise die Beobachtung von V2.1 erzeugt (`p_obs_mode = 1`: Reihenfolge, Vorzeichen,
EE-Orientierung in `e_ori`, Clipping, Befund A22). Mit den Standardwerten reproduziert das Modell weiter alle
Originalmodelle exakt (`D0_check_20260924_232429`), D1 bleibt gültig.

**Befunde aus den alten Logs (Läufe 008–014, ohne Zeitstempel, A37):**

- In allen 40-Hz-Läufen mit V2.1 rückte die geloggte Referenz um genau 25 ms pro Schritt vor. Die Referenz
  zählte also Schritte, nicht Zeit. In den V2.2-Läufen 012 und 013 (10 Hz) waren es 0,10–0,11 s pro Schritt,
  also Wandzeit. Die V2.1-Datei im Repo rechnet mit Wandzeit und einer anderen Bahn. Sie ist nicht der Stand, der
  am 28.04. lief (Log-Hash `d9472bf` ist ein Stand vom Februar, der Laptop lief mit nicht committetem Code).
- Die tatsächliche Schrittdauer lässt sich aus Gelenkweg pro Schritt geteilt durch Geschwindigkeit schätzen:
  etwa 130–160 ms in den 40-Hz-Läufen (Lauf 008: 75 ms). Bei Lauf 012 mit gemessenen 116 ms ergibt dieselbe
  Schätzung 80–100 ms, sie liegt also eher zu niedrig. Die Referenz lief damit mit etwa 15–19 % der
  Trainingsgeschwindigkeit. Das ist die Herkunft der „≈ 18 %“ im Paper.
- V2.1 rechnete den Rate Limiter fest mit 25 ms pro Schritt, auch bei längeren Schritten.

**Teil A, Faktorversuch** (feste Basis, deterministisch, PPO_base und CDR2-4, je 24 Kombinationen):

| Faktor | Anteil der Läufe über 0,4 m |
|---|---|
| Beobachtung wie im Training | 0 von 24 |
| Beobachtung wie V2.1 | 22 von 24 |
| Rate 40 Hz / 7,1 Hz | 8 von 16 / 14 von 32 |
| Referenz Wandzeit / pro Schritt | 11 von 24 / 11 von 24 |
| Faktor 1,0 / 0,5 | 11 von 24 / 11 von 24 |

Ohne den Beobachtungsfehler überschreitet keine Kombination aus Rate, Referenztakt, Skalierung und Rate Limiter
die 0,4 m. Mit dem Beobachtungsfehler scheitern beide Agenten schon bei 40 Hz, Wandzeit und Faktor 1,0
(PPO_base nach 59, CDR2-4 nach 66 Schritten). Rate, Referenztakt und Skalierung verändern den Fehler, führen
allein aber nicht zum Abbruch.

**Teil B, Nachbau der V2.1-Läufe** (alle V2.1-Eigenheiten, Faktor wie im Lauf, Schritt 100/125/150 ms):
20 von 21 Nachbauten überschreiten 0,4 m. Schritte bis dahin: PPO_base 36–56 (Hardware 32–44), CDR2-4 40–112
(Hardware 34–67). Die Reihenfolge stimmt, ein größerer Faktor führt früher zum Stopp. Der EE-Fehler pro Schritt
weicht im Mittel um 5–13 cm vom Log ab. Die Simulation stoppt etwas später als die Hardware. Wahrscheinliche
Ursachen sind die Latenz innerhalb eines Schritts und die Servodynamik, beide sind nicht modelliert.

**Teil D, Nachbau der V2.2-Läufe mit ppo_10hz** (gleicher Beobachtungsfehler, Wandzeit, 110 ms): Lauf 012
(Faktor 0,6) läuft wie auf der Hardware durch, größter Fehler 0,294 m (Hardware 0,308 m). Mit Faktor 1,0 stoppt
die Simulation nach 25 Schritten. Auf dem Laptop stoppte ein Lauf mit 1,0 nach 27 Schritten (A40), Lauf 013 lief
dagegen durch. Faktor 0,5 läuft durch, 0,7 knapp (0,38 m). Ohne Beobachtungsfehler laufen alle durch.

**Teil C, Ratenreihe** (frei schwebende Basis, 65 kg, korrekte Beobachtung, vier Agenten, 25–175 ms,
Referenz nach Wandzeit oder pro Schritt um die Trainingsschrittweite): keiner der 56 Läufe über 0,4 m. Mit
Wandzeit steigt das EE-MSE der 40-Hz-Agenten von 25 auf 175 ms etwa um den Faktor 1,5 bis 1,6 (CDR2-4,
Optimized) bzw. 40 (PPO_base, auf 0,0046 m²). Die Referenz pro Schritt erhöht den Fehler stärker, weil Position und
Geschwindigkeitsreferenz dann nicht mehr zusammenpassen.

**Teil E, Vorhersage für die Laborreihe** (V2.4: korrekte Beobachtung, Wandzeit, Faktor 1,0, Rate Limiter mit
der Trainingsschrittweite, feste Basis), festgehalten am 24.09.2026 vor dem Labortermin:

| Agent (Bedingung) | 100 ms | 125 ms | 140 ms | 175 ms |
|---|---|---|---|---|
| CDR2-4 (`T40_cdr_nom`) | 0,177 m | 0,252 m | 0,264 m | 0,264 m |
| PPO_base (`T40_ppo_nom`) | 0,079 m | 0,107 m | 0,126 m | 0,170 m |
| ppo_10hz (`T10_nom`) | 0,163 m | 0,162 m | 0,161 m | 0,159 m |

Werte: größter EE-Fehler. Alle Läufe kommen ohne OOD-Stopp durch. Die Simulation enthält weder Servodynamik noch
Latenz noch Jitter. Die Vorhersage lautet deshalb nur: Mit korrekter Beobachtung sollten beide 40-Hz-Agenten die
Bahn ohne OOD-Stopp abfahren, der größte Fehler etwa 0,1–0,3 m.

**Folgerung:** In Simulation erklärt der Beobachtungsfehler von V2.1 und V2.2 (A22) das Scheitern der
40-Hz-Agenten. Die Ratenreduktion allein erklärt es nicht (A48). Die Laborreihe prüft das auf der Hardware.

### D3 Frei schwebende gegen fest montierte Basis ✅ 24.09.2026

Skript `desktop_d3_base.m`, Ergebnis `data/simulation/desktop/D3_base_20260924_234944.*`. Feste Basis über
`base_mass = 1e9` (Basisdrehung dann unter 6·10⁻⁹ rad). Deterministisch, korrekte Beobachtung, Wandzeit.

**Teil A** (Halbkreis 8,5 s, 65 kg gegen feste Basis):

| Agent | Rate | EE-MSE frei [m²] | EE-MSE fest [m²] | Änderung | Basisdrehung frei, max |
|---|---|---|---|---|---|
| CDR2-4 | 40 Hz | 0,00347 | 0,00244 | −30 % | 0,012 rad |
| CDR2-4 | 10 Hz | 0,00404 | 0,00320 | −21 % | 0,016 rad |
| Optimized | 40 Hz | 0,00357 | 0,00212 | −41 % | 0,018 rad |
| Optimized | 10 Hz | 0,00449 | 0,00290 | −35 % | 0,017 rad |
| PPO_base | 40 Hz | 0,00011 | 0,00033 | +209 % | 0,056 rad |
| PPO_base | 10 Hz | 0,00127 | 0,00112 | −12 % | 0,051 rad |
| ppo_10hz | 40 Hz | 0,00431 | 0,00307 | −29 % | 0,011 rad |
| ppo_10hz | 10 Hz | 0,00438 | 0,00298 | −32 % | 0,013 rad |

- Auf fester Basis ist der EE-Fehler meist 12 bis 41 % kleiner. Die Rückwirkung der Armbewegung auf die Basis
  trägt in Simulation also merklich zum Fehler bei. Ein Test auf fester Basis ist damit eher etwas leichter als
  der frei schwebende Fall.
- Ausnahme PPO_base bei 40 Hz: Der Fehler steigt auf fester Basis, bleibt aber sehr klein (0,00033 m²). Die
  Rohaktionen unterscheiden sich stark (RMS-Differenz 0,50 gegen unter 0,013 bei den anderen Agenten). PPO_base
  reagiert also deutlich auf die Basisbeobachtungen, die auf fester Basis null sind.
- Alle Episoden laufen durch. Die Basisverschiebung ist nicht geloggt (`q_base` ist kein Positionssignal).

**Teil B, Zerlegung der Lücke an den Hardwareläufen 074–078** (V2.3: ppo_10hz, korrekte Beobachtung, Bahn 17 s,
Faktor 0,35, Wandzeit, Rate Limiter 0,1 s pro Schritt, Schritt 165 und 175 ms, Hardware im Mittel 174 ms):

| Stufe | RMS-Fehler | größter Fehler | Abweichung vom Hardwareverlauf (RMS) |
|---|---|---|---|
| Simulation, Basis frei (65 kg) | 65,5 mm | 193–195 mm | 15,7–16,4 mm |
| Simulation, Basis fest | 54,3–54,8 mm | 160–162 mm | 3,6–4,5 mm |
| Hardware 074–078 | 52,7 ± 0,9 mm | 156 mm | – |

- Die Simulation mit fester Basis und den Einstellungen von V2.3 trifft die Hardware auf etwa 4 mm genau (RMS
  des Unterschieds der Fehlerverläufe), der RMS-Fehler weicht um 2 mm ab (4 %).
- Von der Lücke zwischen frei schwebender Simulation und Hardware (12,8 mm RMS) entfallen damit etwa 11 mm auf die
  fehlende Basisdynamik und etwa 2 mm auf Schnittstelle, Servodynamik und Timing.
- Einschränkung: nur ein Agent, eine Bahn und fünf Läufe mit Faktor 0,35 und halber Bahngeschwindigkeit. Für
  Faktor 1,0 und die Trainingsbahn liefert die Laborreihe (`T10_nom`) den Vergleich.

### D4 Set-Point über die Laborstartliste ✅ 24.09.2026

Skripte `desktop_build_point_models.m`, `desktop_run_setpoint.m`, `desktop_d4_setpoint.m`. Ergebnis
`data/simulation/desktop/D4_setpoint_20260924_235241.*` (4 Prüfläufe, 120 Episoden).

- Modelle: `SK_point_fixed` (Kopie des Trainingsmodells von `test_agent_fixed1`: Basis verschweißt, Schwerkraft,
  Basisbeobachtungen null, Rauschblock) und `SK_point` (frei schwebend, Basismasse und Startpose als Variablen).
  `SK_point` reproduziert das Original bei 650 kg (Abweichung 8·10⁻⁹ m). Das Original `_point_fixed` läuft mit
  `sim()` nicht, weil der Rauschblock eine kontinuierliche Abtastzeit erbt. In der Kopie rechnet er im
  Agenten-Takt 0,1 s. `SK_point_fixed` ist wiederholbar (3·10⁻⁹ m).
- Starts S00–S14 der Laborliste, Ziel [0,479, −0,005, 1,136] m, 25 s, Abbruch wie im Training. Dazu der alte
  OOD-Start [0 −90 0 0 0 0 0]° von Table VII. Deterministisch, außer wo angegeben.

| Satz | Endfehler < 50 mm | Erfolgsabbruch | Endfehler Median | Endfehler max |
|---|---|---|---|---|
| fixed1 im Trainingsmodell (fest, Schwerkraft) | 15/15 | 13/15 | 18 mm | 22 mm |
| fixed1 wie oben mit Trainingsrauschen (Phase 4) | 15/15 | 13/15 | 18 mm | 22 mm |
| fixed1 frei schwebend, 65 kg | 12/15 | 4/15 | 25 mm | 131 mm |
| rand2 frei schwebend, 65 kg | 3/15 | 0/15 | 72 mm | 134 mm |
| rand2 frei schwebend, 650 kg | 2/15 | 0/15 | 67 mm | 74 mm |

- **Hardware-Agent `fixed1` in seinem Trainingsmodell:** Alle 15 Starts enden unter 50 mm (15–22 mm), 13 mit
  Erfolgsabbruch nach 2,7–11,6 s. S00 und S04 bleiben bei 22 mm stehen, J6 steht dort an der Trainingsgrenze
  115,2°. Das deckt sich mit dem kinematischen Trockenlauf vom 22.09. Das Rauschen ändert daran nichts. Kein
  Gelenk überschreitet eine Hardwaregrenze des Gen3 (J4 höchstens 130°, J6 höchstens 115,2°).
- **Vorhersage für die Laborreihe `S10_nom`:** In Simulation erreichen alle 15 Starts das 50-mm-Kriterium, der
  Endfehler liegt unabhängig von $d_0$ bei 15–22 mm. S00 und S04 (und eventuell S08, S09, S14 mit J6 an der
  Grenze) sind die Kandidaten für Stillstand knapp über dem Trainingskriterium.
- **`fixed1` auf frei schwebender Basis:** Die fernen Starts (S11, S13, S14) verfehlen 50 mm. Der Agent ist auf
  fester Basis trainiert und nicht für die Basisrückwirkung ausgelegt.
- **Table VII (A24):** `rand2` vom alten OOD-Start mit 650 kg reproduziert die Tabelle deterministisch
  (Setzzeit 11,98 s, nächste Annäherung 11,7 mm, Endfehler 25 mm). Stochastisch (20 Seeds) liegt der Endfehler bei
  37 ± 15 mm, 15 von 20 enden unter 50 mm, alle kommen dem Ziel näher als 50 mm (nächste Annäherung 19 ± 12 mm).
  **Mit 65 kg scheitert derselbe Agent**: Endfehler 190 ± 30 mm, keine Episode unter 50 mm, Basisdrehung
  0,37 rad. Von den 15 Starts der Laborliste erreicht `rand2` auch mit 650 kg nur 2 das 50-mm-Kriterium. Table VII
  gilt damit nur für 650 kg und diesen einen Start.

### D5 CDR-Agenten nachrechnen ✅ 25.09.2026

Skript `desktop_d5_cdr.m`, Ergebnis `data/simulation/desktop/D5_cdr_20260925_002408.*` (528 Episoden, 24 min).
Acht Agenten bei 40 Hz in `SK_desktop` (65 kg), je Bedingung eine deterministische Episode und 10 stochastische
Seeds. Bedingungen wie in E1: nominal, gespiegelter Halbkreis, 25 % Basismasse, Verzögerung 2 Schritte, Dämpfung
×2, alle drei Störungen zusammen.

**Die Annahmen treffen zu.** Mit `Optimized.mat` als Baseline ohne CDR, `Trajectory2.mat` als CDR-1-Agent und dem
an x gespiegelten Halbkreis (`mirror_x`) ergeben sich die Werte aus E1 fast genau (stochastisch, 10 Seeds):

| E1-Tabelle | Agent | K1 [m²] E1 / D5 | K2 [m] E1 / D5 | K3 [rad] E1 / D5 | K7 E1 / D5 |
|---|---|---|---|---|---|
| `tab:cdr1_results` | No-CDR (`Optimized`) | 0,0691 / 0,0694 | 0,3485 / 0,3444 | 0,0117 / 0,0115 | 211 / 207 |
| `tab:cdr1_results` | CDR-1 (`Trajectory2`) | 0,0352 / 0,0354 | 0,2624 / 0,2602 | 0,0168 / 0,0297 | −97 / −97 |
| `tab:cdr_combined_results` | No-CDR (`Optimized`) | 0,0055 / 0,0056 | 0,2103 / 0,2069 | 0,0120 / 0,0117 | 1042 / 1066 |
| `tab:cdr_combined_results` | CDR-2 bis 4 (`CDR2-4`) | 0,0049 / 0,0051 | 0,1970 / 0,1990 | 0,0119 / 0,0118 | 1176 / 1168 |

Nur K3 des CDR-1-Agenten weicht ab. Die übrigen Werte liegen innerhalb weniger Prozent. Die CDR-Zahlen des Papers
haben damit Rohdaten und ein Skript.

Einzelbefunde (stochastisch, jeweils gegen `Optimized` unter derselben Störung):

- **CDR-1** (`Trajectory2`, gespiegelte Bahn): K1 −49 %, K2 −24 %. Bestätigt „approximately halved“.
  `Trajectory.mat` (erste Fassung) ist auf der gespiegelten Bahn schlechter als die Baseline (0,081 m²).
- **CDR-2** (`Mass_inertia`, 25 % Masse): K1 −13 %, Return +8 %. Bestätigt „small but consistent“.
- **CDR-3** (`Actuator_delay`, 2 Schritte): Basisorientierung (K3) −32 %, aber K1 +85 % (0,0076 gegen
  0,0041 m²). Der Agent ist auch ohne Verzögerung fast doppelt so ungenau (0,0076 gegen 0,0039 m²). „slightly
  lower tracking accuracy“ untertreibt das.
- **CDR-4** (`Friction`, Dämpfung ×2): Für jeden Agenten ist das Ergebnis mit doppelter Dämpfung identisch mit dem
  nominalen. Die Dämpfung wirkt im Modell nicht auf die Bewegung (A23). „largely neutral“ ist damit strukturell
  und kein Lerneffekt.
- **CDR-2 bis 4** (`CDR2-4`, alle drei Störungen): K1 −9 % stochastisch, −10 % deterministisch. Das Paper nennt
  ≈ 11 % (E1: −10,9 %).
- Die Störung „2 Schritte Verzögerung“ ändert K1 bei allen Agenten um höchstens 5 %. 25 % Basismasse erhöht K1
  bei Optimized, Friction, Mass_inertia, CDR2-4 und Trajectory2 um 28 bis 57 %, bei den nominal schwächeren
  Agenten (Actuator_delay, Trajectory, CDR1-4) um höchstens 12 %.
- `CDR1-4` (alle vier Merkmale) ist schon nominal deutlich schlechter (0,0096 m²). Das passt zu E1, wo dieser Agent
  bewusst weggelassen wurde.

### D6 NaN/Inf und Schritt-Reward ✅ 25.09.2026

Skript `desktop_d6_divergence.m`, Ergebnis `data/simulation/desktop/D6_divergence_20260925_010325.*` (300 Episoden).
Fünf untrainierte PPO-Agenten (Standardnetze, zufällige Gewichte, Seeds 1–5) mit je 40 stochastischen Episoden
entsprechen dem Anfang des Trainings. Dazu CDR2-4 mit 100 stochastischen Episoden unter Störungen der CDR-Phase 4
(Basismasse 65 kg · max(0,5; 1 + 0,65 randn), Verzögerung 1–3 Schritte). 40-Hz-Modell, Halbkreis 8,5 s.

| Satz | Episoden | NaN/Inf | Abbruch EE-Fehler > 0,5 m | Schritt-Reward Median (10–90 %) | Schritte unter −13 |
|---|---|---|---|---|---|
| untrainiert | 200 | 0 | 28 (14 %) | −1,3 (−4,0 bis +1,6) | 0,04 % |
| CDR2-4, Phase 4 | 100 | 0 | 0 | +5,6 (+1,4 bis +5,9) | 0 |

- In keiner der 300 Episoden trat NaN oder Inf auf, auch kein Solverabbruch. Die Gelenke sind bewegungsgesteuert,
  der Solver integriert nur die Basisdynamik. Die Häufigkeit im eigentlichen Training bleibt unbekannt (R1.1).
- Die Aussage in Sec. III-C, −50 sei „etwa das Vierfache des typischen geclippten Schritt-Rewards (≈ −13)“, ist so
  nicht haltbar. Typisch sind −1 bis −4 zu Beginn des Trainings und positive Werte beim trainierten Agenten. Die
  größte mögliche geclippte Strafe pro Schritt ist 21,7. Richtig wäre etwa: „−50 übersteigt die größte geclippte
  Strafe eines Schritts (21,7) und ist ein Vielfaches der typischen Schrittwerte“ (A52).

### D7 Beobachtungsrauschen ✅ 25.09.2026

Skript `desktop_d7_noise.m`, Ergebnis `data/simulation/desktop/D7_noise_20260925_011209.*` (182 Episoden). Dafür hat
`SK_desktop` einen Rauscheingang im Block „Obs Transform“ bekommen (`p_obs_noise`, Zufallszahlen aus MATLAB, damit
Seeds wirken). Deterministische Policy, 65 kg, Halbkreis 8,5 s, 10 Seeds je Stufe. Stufe × Rauschstärken aus dem
Set-Point-Training (EE-Fehler 3 mm, EE-Geschwindigkeit 10 mm/s, Gelenkwinkel 2 mrad, Gelenkgeschwindigkeit
10 mrad/s, Basisgrößen 5 mm/s bzw. 5 mrad/s, Orientierung 3 mrad). Keiner der beiden Agenten wurde mit Rauschen
trainiert.

| Stufe (alle Gruppen) | CDR2-4 (40 Hz), EE-MSE | Änderung | ppo_10hz (10 Hz), EE-MSE | Änderung |
|---|---|---|---|---|
| 0 | 0,00347 | – | 0,00438 | – |
| 1 | 0,00347 | +0,1 % | 0,00437 | −0,4 % |
| 2 | 0,00350 | +0,9 % | 0,00436 | −0,6 % |
| 5 | 0,00367 | +5,8 % | 0,00442 | +0,7 % |
| 10 | 0,00422 | +21 % | 0,00490 | +12 % |

- Kein Abbruch, kein Lauf über 0,4 m. Bis zum Fünffachen des Trainingsrauschens ändert sich der Fehler um
  höchstens 6 %.
- Einzelne Gruppen auf Stufe 5: Am empfindlichsten ist CDR2-4 auf Rauschen der EE-Geschwindigkeit (+4 %) und der
  Gelenkgeschwindigkeiten (+1 %). Rauschen auf EE-Position, Gelenkwinkeln und Basisgrößen ändert unter 0,5 %.
- Für R1.7 und Sec. VII: In Simulation sind die Agenten gegen Messrauschen in realistischer Größe unempfindlich.
  Nicht getestet sind systematische Fehler (Versatz, Verzögerung der Messung) und Fehler der Zielschätzung.

### D8 Fig. 5 neu und der Einbruch im letzten Drittel (A51) ✅ 25.09.2026

Fig. 5 soll laut Paper den abgestimmten PPO-Agenten (`Optimized.mat`, Bayes, ohne CDR) zeigen. Nach Angabe des
Nutzers folgte er früher der Bahn bis zum Ende und brach später im letzten Drittel ein.

Befunde (25.09.2026):

- Alle verfügbaren Modellstände ab dem 09.04. (Archiv und Git, darunter der Stand vom 20.04.) haben dieselben
  Sättigungen, Filter, Rate Limiter und Geometrie. In jedem folgt `Optimized` bis etwa 5 s auf 2 mm genau und fällt
  dann zurück (3 cm bei 6 s, 8 cm bei 7 s, 18 cm am Ende). Eine Modelländerung erklärt den Einbruch nicht.
- `Optimized.mat` ist seit dem 21.04. unverändert (gleicher Git-Blob).
- `calculate_kpi_spacekinova.m` lud am 21.04. fest `SpaceKinova_PPO_agent_motionprofile.mat` (Basis-PPO). Die
  Abbildung `Figures/Simulation/optimized_ppo_circular_trajectoy_average.png` vom 21.04. deckt sich mit dem Basis-PPO
  (Endpunkt [0,004; 1,289] m, Abweichung höchstens 1,5 cm), nicht mit `Optimized` (Endpunkt [0,178; 1,323] m). Die
  „gute“ Abbildung zeigt damit sehr wahrscheinlich das Basis-PPO und ist falsch beschriftet.
- Im letzten Drittel lässt `Optimized` das Schultergelenk J2 fast stehen (im Mittel −0,04 statt −0,21 rad/s beim
  Basis-PPO). J2 erreicht dadurch nur −40° statt −68°. J6 setzt spät ein und ist auf 0,1 rad/s begrenzt. Der
  Endeffektor fährt dann fast senkrecht nach unten und kreiselt bei z ≈ 1,32 m. Auf der Dreiecksbahn tritt derselbe
  Einbruch auf. CDR2-4 und ppo_10hz zeigen ihn ebenfalls, nur das Basis-PPO nicht. Es ist gelerntes Verhalten.
  Eine plausible, aber nicht belegte Erklärung ist, dass die Policy die Schulter ruhig hält, um die Basis zu schonen.
- Neue Daten: `desktop_d8_fig5_data.m` (50 stochastische Episoden je Agent), Entwurf der Abbildung mit
  `fig/src/plot_fig5_paths.py` im Paper-Repo (`fig/ppo40hz_paths.pdf`). Seit 25.09. im Paper: Fig. 5 zeigt alle drei
  Agenten, der neue Unterabschnitt IV-D (`sec:sim_tracking`) beschreibt den Einbruch. Offen bleibt die Ursache.

### D9 Sprungtest ✅ 25.09.2026

Skript `desktop_d9_steptest.m`, Ergebnis `data/simulation/desktop/D9_steptest.csv`. Log
`data/hardware/kinova_velocity_test_log.mat` aus `hardware/playback/kinova_test.m`: jedes Gelenk mit ±5 °/s für
1,5 s. Die Zeitachse beginnt in jedem Abschnitt neu. Die Schleife lief mit 40–180 ms pro Schritt (Median 53 ms)
statt der geplanten 10 ms.

- Verstärkung (Geschwindigkeit aus den Winkeln / Befehl): 1,00 im Median (0,987–1,010) bei allen 14 Sprüngen.
- Zeit bis 50 % des Befehls 55–104 ms, bis 90 % 100–168 ms. Die Auflösung liegt bei etwa einem Schritt (50 ms).
- Bei der Frequenz der Halbkreisbahn (1/17 Hz) entspricht das 1–2° Phasenverzug.
- H2 („near-unity amplitude retention and negligible phase lag“) ist damit für die Bahnfrequenz belegt. Die
  Verzögerung von 50–150 ms ist gegenüber dem Regeltakt aber nicht klein (ein halber bis ganzer Schritt bei 10 Hz)
  und sollte im Text genannt werden.
- H1 (Clip-Schwelle ohne Wirkung) lässt sich mit diesem Log nicht prüfen. Dafür fehlen Daten.

### D10 Gleiches Agentenpaar 40/10 Hz ✅ 25.09.2026 (Seeds 0 bis 2)

**Ergebnis über drei Seeds** (gleiches Budget von 340 000 Agentenschritten: 1000 Episoden bei 40 Hz, 4000 bei
10 Hz). Seeds 1 und 2 mit `desktop_d10_run([1 2], 4000, false)`, Auswertung
`data/simulation/desktop/D10_eval_seed12_ep4000_20260925_123422.*`, zusammen mit den beiden Seed-0-Dateien unten.
EE-MSE [m²], Mittelwert ± Std über die drei Seeds, jeder Seed gemittelt über den Satz:

| Satz | 40-Hz-Agenten bei 40 Hz | 10-Hz-Agenten bei 10 Hz | 40-Hz-Agenten bei 10 Hz | 10-Hz-Agenten bei 40 Hz |
|---|---|---|---|---|
| `mass` (deterministisch, 20 Massenziehungen) | 0,00379 ± 0,00074 | 0,00382 ± 0,00117 | 0,00393 ± 0,00037 | 0,00455 ± 0,00225 |
| `nominal` (deterministisch) | 0,00393 ± 0,00073 | 0,00394 ± 0,00123 | 0,00406 ± 0,00036 | 0,00467 ± 0,00233 |
| `stoch` (20 Seeds der Policy) | 0,00470 ± 0,00027 | 0,00404 ± 0,00133 | 0,00483 ± 0,00024 | 0,00485 ± 0,00264 |

Je Seed (`mass`): Verhältnis 10-Hz- zu 40-Hz-Agent bei eigener Rate 1,11 / 0,78 / 1,12. 40-Hz-Agent bei 10 Hz
−7 / +10 / +13 %. 10-Hz-Agent bei 40 Hz +39 / 0 / +7 %.

- **Bei eigener Rate sind beide Raten im Mittel gleich genau** (0,00379 gegen 0,00382 m²). Die Streuung zwischen
  den Seeds (Std 20–30 %) ist viel größer als der Unterschied zwischen den Raten. Seed 0 (+11 % für 10 Hz) war
  kein typischer Fall.
- Die fremde Rate kostet im Mittel wenig: 40-Hz-Agenten verlieren bei 10 Hz 4 %, 10-Hz-Agenten bei 40 Hz 19 %.
  Den großen Wert trägt ein einzelner Seed (+39 %).
- Alle 688 Episoden der D10-Auswertung laufen ohne Abbruch durch. Der größte Fehler liegt bei den fairen Agenten
  unter 0,26 m.
- Alle Lernkurven sind am Ende fast flach (Anstieg über die letzten 5 % der Episoden unter 1 %).
- Deutung für R1.6: Bei gleichem Training und gleichem Schrittbudget ist 10 Hz in Simulation für diese Aufgabe
  nicht schlechter als 40 Hz. Das stützt die Empfehlung, mit der erreichbaren Rate zu trainieren. Es spricht
  zusammen mit D1 und D2 dagegen, dass die niedrigere Rate allein die Hardwareläufe der 40-Hz-Agenten scheitern
  ließ.

Aufbau und Einzelheiten:

- Zwei PPO-Agenten mit demselben Skript, demselben Modell aus D0, den Bayes-Hyperparametern und 65 kg, ohne CDR.
  Verschieden sind nur Agentenrate und Solver-Schritt. 1000 Episoden, Seed 0, auf Wunsch Seeds 0 bis 2.
- Damit ist der Ratenvergleich frei von den Störgrößen aus A21 und A33.
- Entscheidungen des Nutzers vom 25.09.: erst Seed 0, Seeds 1 und 2 nach dem Ergebnis. Hyperparameter pro
  Agentenschritt gleich (γ = 0,99, Experience Horizon 600). Bei 10 Hz ist ihr zeitlicher Horizont damit viermal
  länger (γ etwa 10 s statt 2,5 s), das muss im Text stehen. Über die Hardware wird nach dem Ergebnis entschieden.
- Auf dem Deploy-Laptop ist seit dem 25.09. R2026a installiert (Nutzerangabe). Die Kompatibilitätsfrage mit R2025b
  entfällt damit. Das Paper nennt für die alten Läufe weiter R2025b (`meta.matlabVersion`).
- Skripte: `desktop_d10_train.m` (ein Agent, Optionen wie `Optimized.mat` ausgelesen: 600 / 200 / 10 Epochen /
  Clip 0,2 / γ 0,99 / GAE 0,95 / Entropie 1e-3 / 5,7e-5 und 1e-3 / 2 × 128 ReLU, 40 Hz mit Solver 5 ms, 10 Hz mit
  20 ms, async mit 8 Workern), `desktop_d10_run.m` (beide Trainings, dann Auswertung mit `desktop_d1_rates` und
  denselben Sätzen wie D1, `Optimized` als Referenz). Agenten in `SavedAgents/MotionProfile/D10/`, Lernkurven und
  Auswertung in `data/simulation/desktop/D10_*`.
- Kurztest vom 25.09.: 16 Episoden bei 40 Hz in 1,1 min einschließlich Kompilieren. Asynchrones Training ist auch
  mit festem Seed nicht bitgenau wiederholbar.

**Ergebnis Seed 0 allein**, vor den Seeds 1 und 2 geschrieben. Die Deutung am Ende ist durch das Ergebnis über
drei Seeds oben überholt (Trainingszeit 7,1 min bei 40 Hz, 1,9 min bei 10 Hz mit 1000 und 8,1 min mit 4000 Episoden).
Dateien: Agenten `SavedAgents/MotionProfile/D10/D10_ppo_40hz_seed0.mat`, `D10_ppo_10hz_seed0.mat`,
`D10_ppo_10hz_seed0_ep4000.mat`. Lernkurven `data/simulation/desktop/D10_*_train_*.csv`, Auswertung
`D10_eval_seed0_20260925_111352.*` (beide 1000-Episoden-Agenten und `Optimized`) und
`D10_eval_seed0_ep4000_20260925_112843.*`, Konsolenlogs `D10_log_*.txt`.

- **Gleiche Episodenzahl ist kein fairer Vergleich.** Bei 10 Hz hat eine Episode 85 statt 340 Schritte. Nach 1000
  Episoden hat der 10-Hz-Agent ein Viertel der Schritte und entsprechend weniger Updates gesehen. Seine Lernkurve
  steigt dann noch deutlich (Mittel der letzten 50 Episoden 141, bis Episode 650 vorzeitige Abbrüche), er ist
  nicht auskonvergiert. Deshalb zusätzlich 4000 Episoden bei 10 Hz, also dasselbe Budget an Agentenschritten
  (340 000) wie 1000 Episoden bei 40 Hz. Diese Kurve flacht ab (letzte 100 Episoden 381, davor 377). Die
  40-Hz-Kurve ist nach 1000 Episoden ebenfalls fast flach (1406, dann 1413).

EE-MSE [m²], deterministisch, Satz `mass` (20 Massenziehungen wie D1, Mittelwert), Solver 5 ms, alle Episoden
ohne Abbruch:

| Agent | trainiert | Schritte im Training | bei 40 Hz | bei 10 Hz | Änderung bei fremder Rate |
|---|---|---|---|---|---|
| D10 40 Hz | 40 Hz, 1000 Ep. | 340 000 | **0,00460** | 0,00427 | −7 % |
| D10 10 Hz | 10 Hz, 4000 Ep. | 340 000 | 0,00710 | **0,00511** | +39 % |
| D10 10 Hz | 10 Hz, 1000 Ep. | 85 000 | 0,01487 | 0,02034 | – |
| Optimized (Referenz) | 40 Hz, Originalmodell | – | 0,00347 | 0,00438 | +26 % |

- Bei jeweils eigener Rate und gleichem Schrittbudget liegt der 10-Hz-Agent um 11 % über dem 40-Hz-Agenten
  (0,00511 gegen 0,00460 m²). Stochastisch sind es 0,00554 gegen 0,00460 m² (+20 %).
- Der D10-40-Hz-Agent wird bei 10 Hz nicht schlechter (−7 %), anders als `Optimized` (+26 %) und CDR2-4 (+17 %,
  D1). Mit einem Seed lässt sich nicht sagen, ob das am Agenten oder am Zufall liegt.
- Der 10-Hz-Agent verliert bei 40 Hz 39 %. Er ist auf die längere Wirkung jedes Befehls eingestellt.
- Keine der 344 Episoden bricht ab. Der größte Fehler liegt unter 0,26 m, nur beim nicht auskonvergierten
  1000-Episoden-10-Hz-Agenten bei bis zu 0,37 m. Der größte Fehler der beiden fairen Agenten liegt nominal mit
  0,19–0,24 m in derselben Größe wie bei `Optimized` (0,18 m). Ob auch die neuen Agenten im letzten Drittel
  zurückfallen (A51), ist nicht ausgewertet.
- J6 ist beim 10-Hz-Agenten deterministisch in 100 % der Schritte gesättigt, beim 40-Hz-Agenten in 74–80 %.
- Deutung für R1.6: Bei gleichem Training und gleichem Schrittbudget ist 10 Hz in Simulation etwas ungenauer als
  40 Hz, scheitert aber nicht. Der Unterschied (11 %) ist kleiner als der zwischen `Optimized` und dem neuen
  40-Hz-Agenten (0,00347 gegen 0,00460 m², 33 %). Seeds 1 und 2 würden zeigen, wie groß die Streuung ist.

### D11 Algorithmenvergleich neu

- PPO, TRPO, PG, DDPG, TD3 und SAC mit Toolbox-Standardwerten wie in Sec. IV-A, Seeds 0 bis 2, je 1000 Episoden.
  Lernkurven und Agenten werden gespeichert.
- Rechenzeit grob: PPO, TRPO und PG je 0,5 bis 1 h, DDPG, TD3 und SAC je 1,5 bis 2,5 h, zusammen etwa 20 bis 30 h.
  Die Läufe laufen nacheinander, weil jeder alle 8 Kerne nutzt. Danach KPI-Auswertung, etwa 1 h.
- Table II und Fig. 4 ändern sich, weil das Modell seit März geändert wurde (A20).
- Erst sinnvoll, wenn die Ausrichtung feststeht. Bei Option A reicht eventuell, im Text „ein Seed“ zu schreiben.

### Nicht geplant

- CDR neu trainieren (A23): Das parametrisierte Modell müsste erst wiederhergestellt werden. Stattdessen D5.
- Bayes-Optimierung wiederholen (A29): etwa 12 h, der Code fehlt, wenig Nutzen.
- Set-Point-Agent mit Gelenkgrenzen gleich den Hardwaregrenzen (A43): bräuchte einen weiteren Labortermin.

## Offene Entscheidungen

- (a) Bedingungen für D1. Vorschlag: gemeinsames 40-Hz-Modell mit 65 kg, nominal und gestört.
- (b) Agent für Table VII. Vorschlag: `test_agent_fixed1` wie auf der Hardware, `test_agent_rand2` als Vergleich.
- (c) Welcher PPO-Agent in Fig. 5 gezeigt wird.
- (d) D10 ja oder nein, und ob das Paar auf die Hardware soll.
- (e) D11 erst nach der Ausrichtung. Der Desktop rechnet dann 1 bis 1,5 Tage durch.

## Reihenfolge (Vorschlag)

1. D0, dann D1 und D2, wenn möglich vor dem Labor. Danach über D10 entscheiden.
2. D3, D4 und D5.
3. D6 bis D9.
4. D10 und D11 nach Entscheidung.

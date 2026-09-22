# Simulationsergebnisse ohne Rohdaten

Die meisten Simulationszahlen des Papers wurden nur als Konsolenausgabe oder Abbildung festgehalten. Die
Rohdaten (logsout der Evaluations-Episoden) wurden nicht gespeichert. Dieser Ordner sammelt die schriftlichen
Quellen, damit die Zahlen im Paper einer Datei zugeordnet werden können.

| Datei | Inhalt | Herkunft |
|---|---|---|
| `kpi_default_agents_20260318.md` | KPI-Ausgaben von `evaluation/calculate_kpi_spacekinova.m` für die sechs Agenten mit Toolbox-Standardwerten („Default Agents“), dazu eine PPO-Ablation. Quelle aller Werte in Table II (`tab:algos`). Die Nummern K1 bis K9 folgen dem Skript, im Paper heißen K7 und K8 jetzt $K_4$ und $K_5$ | Kopie von `BachelorThesis/Literature/kpi_tau.md` vom 18.03.2026, unverändert. Der Name „kpi_tau“ ist irreführend: Die Agenten nutzen das Geschwindigkeitsmodell (Befund A20) |

Weitere Quellen außerhalb der Repos, nur als Text vorhanden (siehe `Reviews/Submission_1/paper_numbers.md` im
Paper-Repo): die CDR-Tabellen im Entwurf `BachelorThesis/DigitalAgentAndRateMismatch/root.tex` und das
Vortragsskript `BachelorThesis/Presi/vortrag_script.md`.

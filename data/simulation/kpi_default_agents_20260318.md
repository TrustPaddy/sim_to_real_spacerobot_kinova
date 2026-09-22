1\) duration



2\) cv



3\) drift

%   kpi struct mit Skalaren:

%   --- EE-Tracking ---

%     K1   EE-Position MSE \[m²]

%     K2   Maximaler EE-Trackingfehler \[m]

%   --- Basisstörung ---

%     K3   Basis-Orientierungsfehler (Mittelwert) \[rad]

%     K4   Mittlere Basiswinkelgeschwindigkeit \[rad/s]

%   --- Effizienz ---

%     K5   Mittlere Leistung \[W] (zeitnormiert)

%     K6   Smoothness / Jerk

%   --- Robustheit ---

%     K7   Mean Episode Return

%     K8   Early-Termination-Rate (0..1)
%     K9   Mittlerer Abbruchzeitpunkt \[s]




Default Agents:

**ppo:** 



&#x09;1) 00:28:37



&#x09;2) 1.0531



&#x09;3) 0.2777

--- KPI-Ergebnisse ---

&#x20;            K1: 0.0015

&#x20;            K2: 0.1866

&#x20;            K3: 0.0169

&#x20;            K4: 0.0078

&#x20;            K5: 0.0920

&#x20;            K6: 0.0515

&#x20;            K7: 638.7711

&#x20;            K8: 0

&#x20;            K9: 8.5000

**td3:**

&#x09;

&#x09;1) 44:26



&#x09;2) 0.7702



&#x09;3) -0.0239 



\--- KPI-Ergebnisse ---

&#x20;            K1: 0.0507

&#x20;            K2: 0.5094

&#x20;            K3: 0.0602

&#x20;            K4: 0.0548

&#x20;            K5: 0.0808

&#x20;            K6: 0.0021

&#x20;            K7: -66.9005

&#x20;            K8: 1

&#x20;            K9: 2.8250

**SAC:**

&#x09;

&#x09;1) 01:04:40



&#x09;2) 0.9189



&#x09;3) -1.1620

--- KPI-Ergebnisse ---

&#x20;            K1: 0.0513

&#x20;            K2: 0.5156

&#x20;            K3: 0.0489

&#x20;            K4: 0.0463

&#x20;            K5: 0.0400

&#x20;            K6: 0.0110

&#x20;            K7: -80.1554

&#x20;            K8: 1

&#x20;            K9: 2.8925





**pg:** 

&#x09;1) 00:17:03

&#x09;

&#x09;2) 0.1769

&#x09;

&#x09;3) -0.1003


--- KPI-Ergebnisse ---

&#x20;            K1: 0.0502

&#x20;            K2: 0.5165

&#x20;            K3: 0.0737

&#x20;            K4: 0.0525

&#x20;            K5: 0.0969

&#x20;            K6: 0.0451

&#x20;            K7: -185.0704

&#x20;            K8: 1

&#x20;            K9: 3.4225



**ddpg:** 


	1) 00:40:17

&#x09;

&#x09;2) 5.6775

&#x09;

&#x09;3) -0.6038


--- KPI-Ergebnisse ---

&#x20;            K1: 0.0050

&#x20;            K2: 0.1616

&#x20;            K3: 0.0483

&#x20;            K4: 0.0099

&#x20;            K5: 0.1324

&#x20;            K6: 0.0495

&#x20;            K7: 296.6272

&#x20;            K8: 0

&#x20;            K9: 8.5000

&#x09;

**TRPO:**

	1) 00:47:25

&#x09;

&#x09;2) 0.2631

&#x09;

&#x09;3) 0.4052

--- KPI-Ergebnisse ---

&#x20;            K1: 0.0016

&#x20;            K2: 0.1439

&#x20;            K3: 0.0215

&#x20;            K4: 0.0128

&#x20;            K5: 0.0765

&#x20;            K6: 0.0228

&#x20;            K7: 515.4153

&#x20;            K8: 0

&#x20;            K9: 8.5000


ppo ablation Studie: 

ppo (no gae) - agent.AgentOptions.AdvantageEstimateMethod = 'finite-horizon':
--- KPI-Ergebnisse ---
             K1: 0.0017
             K2: 0.1834
             K3: 0.0190
             K4: 0.0105
             K5: 0.0985
             K6: 0.0362
             K7: 592.1182
             K8: 0
             K9: 8.5000

ppo (no clip) - agent.AgentOptions.ClipFactor = 0.99:
--- KPI-Ergebnisse ---
             K1: 0.0261
             K2: 0.3959
             K3: 0.0163
             K4: 0.0071
             K5: 0.0512
             K6: 0.0613
             K7: -253.8133
             K8: 0
             K9: 8.5000

ppo (no entropy-bonus) - agent.AgentOptions.EntropyLossWeight = 0;
--- KPI-Ergebnisse ---
             K1: 3.5596e-04
             K2: 0.0851
             K3: 0.0185
             K4: 0.0106
             K5: 0.0346
             K6: 0.0414
             K7: 954.0948
             K8: 0
             K9: 8.5000

ppo (1 Training epoch) -  agent.AgentOptions.NumEpoch = 1;
--- KPI-Ergebnisse ---
             K1: 0.0029
             K2: 0.1701
             K3: 0.0854
             K4: 0.0156
             K5: 0.0245
             K6: 0.0198
             K7: 383.0687
             K8: 0
             K9: 8.5000


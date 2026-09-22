function plan = campaign_plan()
%CAMPAIGN_PLAN  Einzige Quelle fuer die Bedingungen der Messkampagne 2026.
%   plan = CAMPAIGN_PLAN() liefert
%     plan.tracking : Struct-Array der Tracking-Bedingungen (deploy_tracking_v24)
%     plan.timing   : Struct-Array der Timing-Messungen (measure_loop_timing)
%     plan.order    : empfohlene Reihenfolge der Laeufe (Zelle mit {condId, Wiederholung})
%   Die Auswertung (evaluation/campaign/analyze_campaign.py) gruppiert nach condId.
%   Beschreibung, Begruendung und Ablauf stehen in hardware/campaign/MESSPLAN.md.
%
%   Bedingungen nur ergaenzen, nie umbenennen: condId steht in Dateinamen und Logs.

a10  = 'SavedAgents/MotionProfile/Circle/PPO/ppo_10hz.mat';
aCdr = 'SavedAgents/MotionProfile/CDR/PPO/CDR2-4.mat';
aPpo = 'SavedAgents/MotionProfile/Circle/PPO/SpaceKinova_PPO_agent_motionprofile.mat';

t = struct('condId', {}, 'agentFile', {}, 'agentLabel', {}, 'rateHz', {}, ...
           'pathDuration', {}, 'cmdScale', {}, 'referenceTiming', {}, ...
           'nRepetitions', {}, 'priority', {}, 'purpose', {});

% Prioritaet 1 = Pflicht, 2 = wenn Zeit bleibt, 3 = nur als Rueckfalloption
t(end+1) = cond('R0_v23_repro', a10, 'PPO 10 Hz', 10, 17.0, 0.35, 2, 1, ...
    'Reproduktion der V2.3-Laeufe 074-078 mit V2.4. Prueft, dass V2.4 dasselbe Verhalten zeigt.');
t(end+1) = cond('T10_nom', a10, 'PPO 10 Hz', 10, 8.5, 1.0, 5, 1, ...
    'Frequenzangepasster Agent auf der Trainingsbahn, ungeskalierte Befehle.');
t(end+1) = cond('T40_cdr_nom', aCdr, 'CDR2-4 40 Hz', 40, 8.5, 1.0, 5, 1, ...
    '40-Hz-Agent (Bayes + CDR) auf der Trainingsbahn, ungeskalierte Befehle, Wandzeit-Referenz.');
t(end+1) = cond('T40_ppo_nom', aPpo, 'PPO 40 Hz (Basis)', 40, 8.5, 1.0, 5, 2, ...
    '40-Hz-Basis-PPO (vor Bayes und CDR), Anschluss an die alten Laeufe 008-011.');
t(end+1) = cond('T10_s05', a10, 'PPO 10 Hz', 10, 8.5, 0.5, 5, 3, ...
    'Rueckfall, falls Faktor 1.0 aus Sicherheitsgruenden nicht gefahren wird. Dann fuer beide Agenten.');
t(end+1) = cond('T40_cdr_s05', aCdr, 'CDR2-4 40 Hz', 40, 8.5, 0.5, 5, 3, ...
    'Rueckfall zu T10_s05 mit demselben Faktor.');
plan.tracking = t;

m = struct('condId', {}, 'mode', {}, 'nCycles', {}, 'nRepetitions', {}, 'agentFile', {}, 'purpose', {});
m(end+1) = struct('condId', 'M_send', 'mode', 'send_only', 'nCycles', 300, 'nRepetitions', 3, ...
    'agentFile', '', 'purpose', 'Nur SendJointSpeedCommand (Null), Table III Zeile 1.');
m(end+1) = struct('condId', 'M_fb', 'mode', 'send_feedback', 'nCycles', 300, 'nRepetitions', 3, ...
    'agentFile', '', 'purpose', 'Senden + RefreshFeedback.');
m(end+1) = struct('condId', 'M_fk', 'mode', 'send_feedback_fk', 'nCycles', 300, 'nRepetitions', 3, ...
    'agentFile', '', 'purpose', 'Senden + Feedback + FK/Jacobi (entspricht dem Playback), Table III Zeile 2.');
m(end+1) = struct('condId', 'M_full', 'mode', 'closed_loop_zero', 'nCycles', 300, 'nRepetitions', 3, ...
    'agentFile', a10, 'purpose', 'Vollstaendige Schleife mit getAction und Logging, Nullbefehl. Table III Zeile 3.');
plan.timing = m;

% Empfohlene Reihenfolge: Timing zuerst (Roboter steht), dann Reproduktion, dann
% die beiden Pflichtbedingungen abwechselnd, damit Drift beide gleich trifft.
o = {};
for i = 1:3
    o(end+1, :) = {'M_send', i}; %#ok<AGROW>
    o(end+1, :) = {'M_fb', i};   %#ok<AGROW>
    o(end+1, :) = {'M_fk', i};   %#ok<AGROW>
    o(end+1, :) = {'M_full', i}; %#ok<AGROW>
end
o(end+1, :) = {'R0_v23_repro', 1};
o(end+1, :) = {'R0_v23_repro', 2};
for i = 1:5
    o(end+1, :) = {'T10_nom', i};     %#ok<AGROW>
    o(end+1, :) = {'T40_cdr_nom', i}; %#ok<AGROW>
end
for i = 1:5
    o(end+1, :) = {'T40_ppo_nom', i}; %#ok<AGROW>
end
plan.order = o;
end

function c = cond(id, agentFile, label, rateHz, pathDuration, cmdScale, nRep, prio, purpose)
c = struct('condId', id, 'agentFile', agentFile, 'agentLabel', label, 'rateHz', rateHz, ...
           'pathDuration', pathDuration, 'cmdScale', cmdScale, 'referenceTiming', 'wall', ...
           'nRepetitions', nRep, 'priority', prio, 'purpose', purpose);
end

function plan = campaign_plan()
%CAMPAIGN_PLAN  Einzige Quelle fuer die Bedingungen der Messkampagne 2026.
%   plan = CAMPAIGN_PLAN() liefert
%     plan.tracking : Struct-Array der Tracking-Bedingungen (deploy_tracking_v24)
%     plan.timing   : Struct-Array der Timing-Messungen (measure_loop_timing)
%     plan.order    : empfohlene Reihenfolge der Laeufe (Zelle mit {condId, Wiederholung})
%     plan.setpoint : Struct-Array der Set-Point-Bedingungen (deploy_setpoint_v24)
%     plan.orderSetpoint : Reihenfolge der Set-Point-Laeufe {condId, startId, Wiederholung}.
%                     Die Startposen stehen in hardware/campaign/setpoint_starts.mat
%                     (make_setpoint_starts).
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

% Set-Point-Reihe (R1.9): ein Ziel (nominales Trainingsziel), feste Startposen ueber
% dem Startabstand d0, je zwei Wiederholungen.
aP2p = 'SavedAgents/MotionProfile/point/test_agent_fixed1.mat';
sp = struct('condId', {}, 'agentFile', {}, 'agentLabel', {}, 'rateHz', {}, 'cmdScale', {}, ...
            'maxDuration', {}, 'eeSource', {}, 'targetMode', {}, 'targetKortex', {}, ...
            'startIds', {}, 'nRepetitions', {}, 'priority', {}, 'purpose', {});
sp(end+1) = spcond('S0_repro073', aP2p, 0.5, 60, 'kortex', 'kortex_fixed', [0.479 -0.105 0.336], ...
    {'S00'}, 1, 1, ['Reproduktion von Lauf 073 mit V2.4 (Kortex-tool_pose, Faktor 0.5, 60 s, Ziel ' ...
    'wie 073). Prueft, dass V2.4 dasselbe Verhalten zeigt wie V3.0.']);
sp(end+1) = spcond('S10_nom', aP2p, 1.0, 25, 'fk', 'nominal', [], 'all', 2, 1, ...
    ['Set-Point-Agent (Fixed-Base-Training) vom Anker und von 14 festen Starts zum nominalen ' ...
    'Trainingsziel, FK-Endeffektor wie im Training, ungeskalierte Befehle, 25 s wie die Episode.']);
sp(end+1) = spcond('S10_s05', aP2p, 0.5, 25, 'fk', 'nominal', [], 'all', 2, 3, ...
    'Rueckfall zu S10_nom mit Faktor 0.5, falls 1.0 aus Sicherheitsgruenden nicht gefahren wird.');
plan.setpoint = sp;

% Reihenfolge: Reproduktion, dann Wiederholung 1 nach aufsteigendem d0 und
% Wiederholung 2 nach absteigendem d0, damit Drift nicht mit d0 zusammenfaellt.
os = {'S0_repro073', 'S00', 1};
startsFile = fullfile(fileparts(mfilename('fullpath')), 'setpoint_starts.mat');
if isfile(startsFile)
    L = load(startsFile);
    [~, ix] = sort([L.S.starts.d0_m]);
    ids = {L.S.starts(ix).id};
    for i = 1:numel(ids), os(end+1, :) = {'S10_nom', ids{i}, 1}; end        %#ok<AGROW>
    for i = numel(ids):-1:1, os(end+1, :) = {'S10_nom', ids{i}, 2}; end     %#ok<AGROW>
end
plan.orderSetpoint = os;
end

function c = spcond(id, agentFile, cmdScale, maxDuration, eeSource, targetMode, targetKortex, ...
                    startIds, nRep, prio, purpose)
c = struct('condId', id, 'agentFile', agentFile, 'agentLabel', 'PPO set-point 10 Hz (fixed base)', ...
           'rateHz', 10, 'cmdScale', cmdScale, 'maxDuration', maxDuration, 'eeSource', eeSource, ...
           'targetMode', targetMode, 'targetKortex', targetKortex, 'startIds', {startIds}, ...
           'nRepetitions', nRep, 'priority', prio, 'purpose', purpose);
end

function c = cond(id, agentFile, label, rateHz, pathDuration, cmdScale, nRep, prio, purpose)
c = struct('condId', id, 'agentFile', agentFile, 'agentLabel', label, 'rateHz', rateHz, ...
           'pathDuration', pathDuration, 'cmdScale', cmdScale, 'referenceTiming', 'wall', ...
           'nRepetitions', nRep, 'priority', prio, 'purpose', purpose);
end

function result = deploy_setpoint_v24(varargin)
%DEPLOY_SETPOINT_V24  Set-Point-Lauf auf dem Kinova Gen3 fuer die Messkampagne (V2.4).
%
%   result = DEPLOY_SETPOINT_V24(condId, startId, repetition)
%       Faehrt eine Bedingung aus campaign_plan().setpoint von der Startpose startId
%       aus hardware/campaign/setpoint_starts.mat (z. B. 'S10_nom', 'S05', 1).
%   result = DEPLOY_SETPOINT_V24(condId, startId, repetition, Name, Value, ...)
%       Ueberschreibt einzelne Einstellungen, z. B. 'dryRun', true.
%
%   Basis ist V3.0 (hardware/deploy/deploy_agent_kinova_point.m, Laeufe 068-073)
%   mit derselben Observation [ep; ev; v_base; w_base; q; dq; e_ori], derselben
%   Trainings-Kette (Saturation aller 7 Gelenke -> IIR -> Rate Limiter), demselben
%   Erfolgskriterium wie im Training (2 cm, 3 cm/s) mit 0.5 s Haltezeit und derselben
%   Schutzschicht. Aenderungen gegenueber V3.0:
%     1. Endeffektor aus der FK der URDF (eeSource 'fk'), wie im Training. V3.0 nutzte
%        die Kortex-tool_pose. Sie liegt wegen des eingetragenen Greifers 0.121 m
%        entlang der Werkzeugachse vor dem Trainingspunkt (kortex_frames).
%        eeSource 'kortex' bildet V3.0 nach (Bedingung S0_repro073).
%     2. Feste Startposen aus setpoint_starts.mat. Anfahrt ueber den Anker in die
%        Startpose, nur fuer im Labor freigegebene Posen (check_setpoint_starts).
%     3. Genau ein Befehlsfaktor cmdScale aus der Bedingung (V3.0: speedScale 0.5,
%        in Lauf 070 0.75).
%     4. Gelenkwinkel werden ab der Startpose stetig abgewickelt. V3.0 hat in jedem
%        Schritt zum Anker abgewickelt, dabei springt ein Endlosgelenk bei 180 deg
%        Abstand zum Anker um 360 deg.
%     5. Rate Limiter mit der Abtastzeit des Agenten, Watchdog max(0.10, 1.5*Ts).
%        V3.0 hatte fest 0.25 s.
%     6. Jeder Stoppschritt wird geloggt, der Grund steht in meta.stopReason.
%     7. Logging wie deploy_tracking_v24 (Teilzeiten, Observation, Befehlskette,
%        gesendeter Befehl in deg/s, Kortex-Pose, Metadaten mit Git, CPU, MD5).
%     8. Hoehenwaechter: Stopp, wenn Handgelenk, Endeffektor oder Greiferspitze
%        tiefer als heightGuard ueber der Montageflaeche liegen.
%     9. OOD-Stopp bei ||ep|| > min(2.0 m, d0 + 0.4 m). V3.0 hatte fest 2.0 m.
%    10. Der Trockenlauf integriert die Gelenke kinematisch (q += dq_cmd * Ts). Das
%        entspricht dem Trainingsmodell (Gelenke bewegungsgesteuert) ohne Verzoegerung
%        und Rauschen. So zeigt sich vor dem Labor, ob der Agent von der Startpose
%        aus konvergiert.
%
%   Logs: data/hardware/campaign/<condId>/<condId>_<startId>_rNN_<Zeitstempel>.mat
%   (Variable "run", Schema sk_campaign_setpoint_v1) und eine Zeile in
%   data/hardware/campaign/campaign_index.csv. Trockenlaeufe landen in _dryrun/.
%
%   SICHERHEIT: E-Stop in Reichweite, Arbeitsraum frei. Vor jeder Anfahrt und vor
%   jedem Lauf fragt das Skript nach ENTER. CTRL+C loest ueber onCleanup
%   Zero-Velocity aus.

%% =========================
%  0) KONFIGURATION
%  =========================
cfg = default_config();
[cfg, overrides] = parse_inputs(cfg, varargin{:});
cfg = derive_config(cfg);

env = campaign_env_meta();
fprintf('\n=== deploy_setpoint_v24 | %s %s r%02d | %s ===\n', cfg.condId, cfg.startId, cfg.repetition, ...
        env.datetimeStart);
if env.gitDirty
    warning('deploy_setpoint_v24:dirty', ['Git-Arbeitsstand hat uncommittete Aenderungen (%s). ' ...
        'Fuer Kampagnenlaeufe vorher committen.'], env.gitHash);
end

%% =========================
%  1) AGENT
%  =========================
assert(isfile(cfg.agentFile), 'Agent-Datei nicht gefunden: %s', cfg.agentFile);
loaded = load(cfg.agentFile);
agent  = loaded.agent;
agent.UseExplorationPolicy = false;
obsInfo = getObservationInfo(agent);
assert(obsInfo.Dimension(1) == cfg.expectedObsDim, ...
    'ObservationDim Mismatch: Agent=%d, Deploy=%d.', obsInfo.Dimension(1), cfg.expectedObsDim);
cfg.agentSampleTime = agent.AgentOptions.SampleTime;
cfg.pipelineTs      = cfg.agentSampleTime;
cfg.agentMd5        = file_md5(cfg.agentFile);
fprintf('Agent: %s (Ts=%.3f s, MD5=%s)\n', cfg.agentFile, cfg.agentSampleTime, cfg.agentMd5);

%% =========================
%  2) ROBOTERMODELL, STARTLISTE, ZIEL
%  =========================
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';
F = kortex_frames(robot_rbt);
cfg.kortexToolOffset = F.toolOffset;

L0 = load(cfg.startsFile);
SL = L0.S;
cfg.startsMd5 = file_md5(cfg.startsFile);
iS = find(strcmp({SL.starts.id}, cfg.startId));
assert(~isempty(iS), 'Startpose %s nicht in %s.', cfg.startId, cfg.startsFile);
sp = SL.starts(iS);
cfg.qStartList = deg2rad(sp.q_deg(:));
cfg.d0List     = sp.d0_m;

switch cfg.targetMode
    case 'nominal'
        assert(strcmp(cfg.eeSource, 'fk'), 'targetMode nominal braucht eeSource fk.');
        cfg.target = SL.target.ee_urdf(:);          % URDF-Frame, Trainingsziel
    case 'kortex_fixed'
        assert(strcmp(cfg.eeSource, 'kortex') && numel(cfg.targetKortex) == 3, ...
               'targetMode kortex_fixed braucht eeSource kortex und targetKortex.');
        cfg.target = cfg.targetKortex(:);           % Kortex-Frame, Werkzeugpunkt (wie V3.0)
end

obsLow  = [-cfg.ePLim*ones(3,1); -cfg.eVLim*ones(3,1); -cfg.vBLim*ones(3,1); -cfg.wBLim*ones(3,1); ...
            cfg.qLim_lower; -cfg.dqLim; -cfg.eOriLim*ones(3,1)];
obsHigh = -obsLow;

%% =========================
%  3) FREIGABE, VERBINDUNG, ANFAHRT
%  =========================
if ~cfg.dryRun && cfg.requireApproval
    check_approval(cfg);
end

apiHandle = [];
if ~cfg.dryRun
    fprintf('== MEX init ==\n');
    apiHandle = kinova_open(cfg);
    cleanupObj = onCleanup(@() kinova_safe_shutdown(apiHandle, cfg.nJ));
    if cfg.doHoming
        input(sprintf('ENTER faehrt ueber den Anker in die Startpose %s (d0 = %.2f m)... ', ...
              cfg.startId, cfg.d0List), 's');
        kinova_move_joints(apiHandle, cfg.qAnchor, cfg);
        if ~strcmp(cfg.startId, 'S00')
            kinova_move_joints(apiHandle, cfg.qStartList, cfg);
        end
        fprintf('  Startpose erreicht\n');
    end
    st0 = kinova_read_state(apiHandle, cfg);
    stStart = st0;
    q_start = unwrap_to(st0.q_rad, cfg.qStartList);
    dev_deg = max(abs(rad2deg(q_start - cfg.qStartList)));
    if dev_deg > cfg.maxStartDev_deg && ~cfg.allowStartOffset
        error('deploy_setpoint_v24:start', ['Startpose weicht um %.1f deg von %s ab (Grenze %.1f deg). ' ...
              'doHoming aktivieren oder allowStartOffset setzen (wird geloggt).'], ...
              dev_deg, cfg.startId, cfg.maxStartDev_deg);
    end
else
    fprintf('\n=== TROCKENLAUF: keine MEX-Verbindung, Gelenke werden kinematisch integriert ===\n');
    q_start = cfg.qStartList;
    dev_deg = 0;
    stStart = [];
end

[ctl0, ~] = control_point(robot_rbt, F, q_start, zeros(cfg.nJ, 1), stStart, cfg);
cfg.d0 = norm(ctl0 - cfg.target);
cfg.oodThreshold = min(cfg.oodMax, cfg.d0 + cfg.oodMargin);
print_safety_summary(cfg);
if ~cfg.dryRun
    input(sprintf('ENTER startet %s %s r%02d (CTRL+C bricht ab)... ', cfg.condId, cfg.startId, ...
          cfg.repetition), 's');
end

%% =========================
%  4) WARM-UP
%  =========================
try
    getAction(agent, {zeros(cfg.expectedObsDim, 1)});
catch ME
    warning('deploy_setpoint_v24:warmup', 'getAction warm-up fehlgeschlagen: %s', ME.message);
end
if ~cfg.dryRun
    for ii = 1:3
        stW = kinova_read_state(apiHandle, cfg);
        if stW.fault, error('deploy_setpoint_v24:fault', 'Fault waehrend Warm-up -- Abbruch.'); end
        pause(0.02);
    end
end

%% =========================
%  5) HAUPTSCHLEIFE
%  =========================
state = struct('yFilt', zeros(cfg.nJ, 1), 'dqPrev', zeros(cfg.nJ, 1));
nStepsMax = ceil(cfg.maxDuration * cfg.rateHz) + 20;
log = init_log(nStepsMax, cfg.nJ, cfg.expectedObsDim);

q = q_start; dq = zeros(cfg.nJ, 1); dq_cmd = zeros(cfg.nJ, 1);
stopReason = 'max_steps'; stopStep = 0; kLast = 0;
convSince = NaN;
fastDry = cfg.dryRun && ~cfg.rateControlInDryRun;

r = rateControl(cfg.rateHz);
cfg.rateControlOverrunAction = char(r.OverrunAction);
try
    reset(r);
catch
    % aeltere MATLAB-Versionen starten rateControl beim Erzeugen
end
tRunStart = tic;
tPrev = 0.0; tLastSend = NaN;

for k = 1:nStepsMax
    s = struct();
    t0 = toc(tRunStart);
    if fastDry, tNow = (k - 1) * cfg.Ts; else, tNow = t0; end
    if k == 1, dtLoop = 0.0; else, dtLoop = tNow - tPrev; end
    tPrev = tNow;
    if isnan(tLastSend), dtSinceSend = NaN; else, dtSinceSend = t0 - tLastSend; end
    s.t_wall = tNow; s.t_ref = tNow; s.dt_loop = dtLoop; s.dt_since_send = dtSinceSend;

    % --- Watchdog ---
    overrun = ~isnan(dtSinceSend) && dtSinceSend > cfg.watchdogTimeout;
    s.timing_overrun = overrun;
    if overrun
        send_zero(apiHandle, cfg);
        if cfg.stopOnTimingOverrun
            s.step_kind = 2;
            log = log_step(log, k, s); kLast = k;
            stopReason = 'watchdog'; stopStep = k;
            break;
        end
    end

    % --- Laufzeit ---
    if tNow >= cfg.maxDuration && k > 1
        send_zero(apiHandle, cfg);
        stopReason = 'timeout'; stopStep = k - 1;
        break;
    end

    % --- Feedback (bzw. kinematische Integration im Trockenlauf) ---
    tA = toc(tRunStart);
    st = [];
    fault = false;
    if ~cfg.dryRun
        st = kinova_read_state(apiHandle, cfg);
        q  = q + wrap_pi(st.q_rad - q);          % stetig abwickeln (Aenderung 4)
        dq = st.dq_rad;
        fault = st.fault;
        s.ee_kortex_real = st.pose_real(1:3);
        s.ee_kortex_ori  = st.pose_real(4:6);
        s.twist_kortex   = st.twist_real;
    elseif k > 1
        q  = q + dq_cmd * dtLoop;
        dq = dq_cmd;
    end
    tB = toc(tRunStart);
    s.dur_feedback = tB - tA;
    s.q = q.'; s.dq = dq.';
    if fault
        send_zero(apiHandle, cfg);
        s.step_kind = 2;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'fault'; stopStep = k;
        break;
    end

    % --- FK, Regelpunkt, Hoehen ---
    [ee_pos, ee_vel, aux] = control_point(robot_rbt, F, q, dq, st, cfg);
    tC = toc(tRunStart);
    s.dur_fk = tC - tB;

    % --- Fehler und Observation (Trainings-Konvention: Ist - Soll) ---
    ep = ee_pos - cfg.target;
    ev = ee_vel;                                 % Sollgeschwindigkeit ist null
    obs_raw = [ep; ev; zeros(3,1); zeros(3,1); q; dq; zeros(3,1)];
    obs = max(min(obs_raw, obsHigh), obsLow);
    tD = toc(tRunStart);
    s.dur_obs = tD - tC;
    s.ee_pos = ee_pos.'; s.ee_vel = ee_vel.'; s.ee_ref = cfg.target.';
    s.ee_fk = aux.ee_fk.'; s.tool_pred_real = aux.tool_pred_real.'; s.ee_kortex_urdf = aux.ee_kortex_urdf.';
    s.min_height = aux.minHeight;
    s.ep = ep.'; s.ep_norm = norm(ep); s.ev = ev.';
    s.obs_raw = obs_raw.'; s.obs = obs.';

    % --- Hoehenwaechter und OOD ---
    if aux.minHeight < cfg.heightGuard
        send_zero(apiHandle, cfg);
        s.step_kind = 2;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'height_guard'; stopStep = k;
        break;
    end
    if norm(ep) > cfg.oodThreshold
        send_zero(apiHandle, cfg);
        s.step_kind = 1;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'ood'; stopStep = k;
        break;
    end

    % --- Konvergenz wie der Erfolgsabbruch im Training, mit Haltezeit ---
    conv = norm(ep) < cfg.convDist && norm(ev) < cfg.convVel;
    if conv
        if isnan(convSince), convSince = tNow; end
    else
        convSince = NaN;
    end
    s.conv_flag = conv;
    s.conv_dwell = ternary(conv, tNow - convSince, 0);
    if cfg.stopOnConvergence && conv && (tNow - convSince) >= cfg.convDwell
        send_zero(apiHandle, cfg);
        s.step_kind = 3;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'converged'; stopStep = k;
        break;
    end

    % --- Agent ---
    action = getAction(agent, {obs});
    tE = toc(tRunStart);
    s.dur_agent = tE - tD;
    dq_raw = max(min(action{1}(:), 1), -1);

    % --- Trainings-Kette ---
    dq_sat = max(min(dq_raw, cfg.satUpper), cfg.satLower);
    state.yFilt = cfg.filt_num * dq_sat + cfg.filt_den * state.yFilt;
    maxDelta = cfg.slewRate * cfg.pipelineTs;
    dq_rl = state.dqPrev + max(min(state.yFilt - state.dqPrev, maxDelta), -maxDelta);
    state.dqPrev = dq_rl;

    % --- Schutzschicht ---
    dq_safe = dq_rl;
    soft = false(cfg.nJ, 1);
    for j = 1:cfg.nJ
        dUp = cfg.qLim_upper(j) - q(j);
        dLo = q(j) - cfg.qLim_lower(j);
        if dUp < cfg.qSoftMargin && dq_safe(j) > 0
            dq_safe(j) = dq_safe(j) * max(dUp / cfg.qSoftMargin, 0); soft(j) = true;
        end
        if dLo < cfg.qSoftMargin && dq_safe(j) < 0
            dq_safe(j) = dq_safe(j) * max(dLo / cfg.qSoftMargin, 0); soft(j) = true;
        end
    end
    capped  = abs(dq_safe) > cfg.dqHwCap;
    dq_safe = max(min(dq_safe, cfg.dqHwCap), -cfg.dqHwCap);

    % --- Einziger Befehlsfaktor ---
    dq_cmd = cfg.cmdScale * dq_safe;
    dq_sent_deg = rad2deg(dq_cmd(:)).';
    tF = toc(tRunStart);
    s.dur_pipeline = tF - tE;
    s.dq_raw = dq_raw.'; s.dq_sat = dq_sat.'; s.dq_filt = state.yFilt.'; s.dq_rl = dq_rl.';
    s.dq_safe = dq_safe.'; s.dq_cmd = dq_cmd.'; s.dq_sent_deg = dq_sent_deg;
    s.softlimit_active = soft.'; s.cap_active = capped.';

    % --- Senden ---
    if ~cfg.dryRun
        errSend = kinova_send_deg(apiHandle, dq_sent_deg, cfg);
    else
        errSend = 0;
    end
    tG = toc(tRunStart);
    s.dur_send = tG - tF;
    tLastSend = tG;
    if errSend ~= 0
        send_zero(apiHandle, cfg);
        s.step_kind = 2;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'send_error'; stopStep = k;
        break;
    end

    % --- Logging ---
    s.step_kind = 0;
    log = log_step(log, k, s); kLast = k;
    tH = toc(tRunStart);
    log.dur_log(k) = tH - tG;

    % --- Takt ---
    if ~cfg.dryRun || cfg.rateControlInDryRun
        waitfor(r);
    end
    log.dur_wait(k) = toc(tRunStart) - tH;
end
send_zero(apiHandle, cfg);

%% =========================
%  6) AUSWERTUNG, METADATEN, SPEICHERN
%  =========================
log  = truncate_log(log, max(kLast, 1));
meta = build_meta(cfg, env, overrides, q_start, dev_deg, stopReason, stopStep, log);

fprintf('\n== Lauf beendet: %s nach %d Schritten (%.2f s) ==\n', stopReason, meta.nSteps, meta.tEnd);
fprintf('   d0 = %.3f m, Endfehler = %.1f mm, naechster Abstand = %.1f mm, Erfolg (50 mm) = %d\n', ...
        meta.d0_m, 1e3 * meta.finalErr_m, 1e3 * meta.minErr_m, meta.success50);
fprintf('   Setzzeit (50 mm) = %.2f s, Pfadeffizienz = %.2f\n', meta.settle50_s, meta.pathEff);
fprintf('   Schleife: Median %.1f ms, wirksamer Faktor %.3f (konfiguriert %.2f)\n', ...
        meta.loopMedian_ms, meta.effFactorMeasured, cfg.cmdScale);

run = struct('schema', 'sk_campaign_setpoint_v1', 'meta', to_char_struct(meta), ...
             'cfg', to_char_struct(cfg), 'log', log);
if cfg.dryRun
    saveDir = fullfile(cfg.saveRoot, '_dryrun', cfg.condId);
else
    saveDir = fullfile(cfg.saveRoot, cfg.condId);
end
if ~isfolder(saveDir), mkdir(saveDir); end
fname = sprintf('%s_%s_r%02d_%s.mat', cfg.condId, cfg.startId, cfg.repetition, ...
                char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
filePath = fullfile(saveDir, fname);
save(filePath, 'run', '-v7');
fprintf('Log gespeichert: %s\n', filePath);

idxRow = struct('file', fname, 'condId', cfg.condId, 'repetition', cfg.repetition, ...
    'datetime', env.datetimeStart, 'dryRun', cfg.dryRun, 'agent', cfg.agentLabel, ...
    'rateHz', cfg.rateHz, 'pathDuration', cfg.maxDuration, 'cmdScale', cfg.cmdScale, ...
    'stopReason', stopReason, 'nSteps', meta.nSteps, 'tEnd', meta.tEnd, 'rmsEp', meta.rmsEp, ...
    'maxEp', meta.maxEp, 'loopMean_ms', meta.loopMean_ms, 'gitHash', env.gitHash, ...
    'scriptVersion', cfg.scriptVersion);
append_index_csv(fullfile(cfg.saveRoot, ternary(cfg.dryRun, 'campaign_index_dryrun.csv', ...
    'campaign_index.csv')), idxRow);

if cfg.showPlots
    plot_run(log, cfg, meta);
end
result = struct('file', filePath, 'meta', meta);
end

%% ========================================================================
%  KONFIGURATION
%  ========================================================================
function cfg = default_config()
cfg = struct();
cfg.scriptName    = 'deploy_setpoint_v24';
cfg.scriptVersion = 'V2.4-SP';
cfg.condId        = 'custom';
cfg.startId       = 'S00';
cfg.repetition    = 0;
cfg.agentLabel    = '';
cfg.purpose       = '';
cfg.operatorNote  = '';

cfg.dryRun              = false;
cfg.rateControlInDryRun = false;     % false: Trockenlauf so schnell wie moeglich, Zeit = k*Ts
cfg.showPlots           = true;

cfg.agentFile      = '';
cfg.expectedObsDim = 29;
cfg.urdfFile   = sk_path('robot', 'SpaceKinova.urdf');
cfg.eeBodyName = 'kinova_end_effector_link';
cfg.startsFile   = sk_path('hardware', 'campaign', 'setpoint_starts.mat');
cfg.approvalFile = sk_path('data', 'hardware', 'campaign', 'setpoint_start_check.csv');
cfg.requireApproval = true;

cfg.kinovaIP         = '192.168.0.10';
cfg.kinovaUser       = 'admin';
cfg.kinovaPassword   = 'admin';
cfg.sessionTimeoutMs = uint32(60000);
cfg.controlTimeoutMs = uint32(200);
cfg.speedCmdDuration = 0;

cfg.nJ         = 7;
cfg.qLim_lower = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];
cfg.qLim_upper = [ 2*pi;  2.41;  2*pi;  2.66;  2.23;  2.01;  2*pi];
cfg.dqLim      = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];
cfg.safetyFactor = 0.75;
cfg.cmdScale     = 1.0;

% Trainings-Kette wie SpaceKinova_MotionProfile_point_fixed.slx
cfg.satUpper = [0.9774; 0.9774; 0.9774; 0.9774; 0.5774; 0.5774; 0.5774];
cfg.satLower = -cfg.satUpper;
cfg.filt_num = 0.05;
cfg.filt_den = 0.95;
cfg.slewRate = 0.5;
cfg.qSoftMargin = deg2rad(10);

cfg.rateHz          = 10;
cfg.maxDuration     = 25;            % Episodenlaenge im Training [s]
cfg.watchdogTimeout = [];            % leer = max(0.10, 1.5*Ts)
cfg.stopOnTimingOverrun = true;

cfg.eeSource     = 'fk';             % 'fk' (URDF, wie Training) oder 'kortex' (tool_pose, wie V3.0)
cfg.targetMode   = 'nominal';        % 'nominal' (Trainingsziel) oder 'kortex_fixed'
cfg.targetKortex = [];               % [x y z] im Kortex-Frame, nur fuer kortex_fixed

% Erfolgsabbruch wie rewardFcn im Trainingsmodell, plus Haltezeit wie V3.0
cfg.stopOnConvergence = true;
cfg.convDist  = 0.02;
cfg.convVel   = 0.03;
cfg.convDwell = 0.5;

cfg.oodMax      = 2.0;               % d_failure im Training
cfg.oodMargin   = 0.4;               % Stopp, wenn ||ep|| > d0 + oodMargin
cfg.heightGuard = 0.05;              % [m] ueber der Montageflaeche

cfg.doHoming         = true;
cfg.qAnchor          = deg2rad([0; 15; 180; -130; 0; 55; 90]);
cfg.homingTimeout_s  = 60;
cfg.homingTol_deg    = 1.0;
cfg.homingSettle_s   = 1.0;
cfg.maxStartDev_deg  = 2.0;
cfg.allowStartOffset = false;

cfg.ePLim = 1.5; cfg.eVLim = 1.0; cfg.vBLim = 0.5; cfg.wBLim = 1.0; cfg.eOriLim = pi;
cfg.saveRoot = sk_path('data', 'hardware', 'campaign');
end

function [cfg, overrides] = parse_inputs(cfg, varargin)
overrides = struct();
assert(numel(varargin) >= 3 && isnumeric(varargin{3}), ...
       'Aufruf: deploy_setpoint_v24(condId, startId, repetition, ...)');
condId = char(varargin{1});
plan = campaign_plan();
c = plan.setpoint(strcmp({plan.setpoint.condId}, condId));
assert(~isempty(c), 'Bedingung %s nicht in campaign_plan().setpoint.', condId);
cfg.condId       = c.condId;
cfg.startId      = char(varargin{2});
cfg.repetition   = varargin{3};
cfg.agentFile    = sk_path(c.agentFile);
cfg.agentLabel   = c.agentLabel;
cfg.rateHz       = c.rateHz;
cfg.cmdScale     = c.cmdScale;
cfg.maxDuration  = c.maxDuration;
cfg.eeSource     = c.eeSource;
cfg.targetMode   = c.targetMode;
cfg.targetKortex = c.targetKortex;
cfg.purpose      = c.purpose;
if ~(ischar(c.startIds) && strcmp(c.startIds, 'all'))
    assert(any(strcmp(c.startIds, cfg.startId)), 'Startpose %s gehoert nicht zu %s.', cfg.startId, condId);
end
rest = varargin(4:end);
assert(mod(numel(rest), 2) == 0, 'Name-Value-Paare erwartet.');
for i = 1:2:numel(rest)
    name = char(rest{i});
    assert(isfield(cfg, name), 'Unbekannte Einstellung: %s', name);
    cfg.(name) = rest{i + 1};
    overrides.(name) = rest{i + 1};
end
cfg.agentFile = char(cfg.agentFile);
end

function cfg = derive_config(cfg)
cfg.Ts      = 1 / cfg.rateHz;
cfg.dqHwCap = cfg.safetyFactor * cfg.dqLim;
if isempty(cfg.watchdogTimeout)
    cfg.watchdogTimeout = max(0.10, 1.5 * cfg.Ts);
end
assert(cfg.cmdScale > 0 && cfg.cmdScale <= 1, 'cmdScale muss in (0, 1] liegen.');
assert(any(strcmp(cfg.eeSource, {'fk', 'kortex'})), 'eeSource unbekannt.');
assert(any(strcmp(cfg.targetMode, {'nominal', 'kortex_fixed'})), 'targetMode unbekannt.');
end

function check_approval(cfg)
assert(isfile(cfg.approvalFile), ['Keine Freigaben gefunden (%s). Startposen zuerst mit ' ...
       'check_setpoint_starts im Labor anfahren und freigeben.'], cfg.approvalFile);
T = readtable(cfg.approvalFile, 'TextType', 'char', 'Delimiter', ',');
ok = strcmp(T.id, cfg.startId) & T.approved == 1 & strcmp(T.listMd5, cfg.startsMd5);
assert(any(ok), ['Startpose %s ist fuer die aktuelle Startliste (MD5 %s) nicht freigegeben. ' ...
       'check_setpoint_starts(''ids'', {''%s''}) ausfuehren.'], cfg.startId, cfg.startsMd5, cfg.startId);
end

%% ========================================================================
%  REGELPUNKT
%  ========================================================================
function [ee_pos, ee_vel, aux] = control_point(rbt, F, q, dq, st, cfg)
% Regelpunkt je nach eeSource. aux enthaelt FK-Endeffektor, vorhergesagte
% Kortex-Werkzeugposition, Kortex-Schaetzung des EE-Punkts und die kleinste Hoehe.
T  = getTransform(rbt, q.', char(cfg.eeBodyName));
J  = geometricJacobian(rbt, q.', char(cfg.eeBodyName));
Tw = getTransform(rbt, q.', 'kinova_spherical_wrist_2_link');
R  = T(1:3, 1:3); p = T(1:3, 4);
Tinv = F.T_urdf_from_real \ eye(4);          % URDF -> Kortex-Frame
aux.ee_fk = p;
toolUrdf = p + R * F.toolOffset;
aux.tool_pred_real = Tinv(1:3, 1:3) * toolUrdf + Tinv(1:3, 4);
gp = R * F.gripperPoints + p;
zBase = F.T_urdf_from_real(3, 4);
aux.minHeight = min([Tw(3, 4), gp(3, :)]) - zBase;
aux.ee_kortex_urdf = nan(3, 1);
if ~isempty(st) && all(isfinite(st.pose_real(1:3)))
    kU = F.T_urdf_from_real * [st.pose_real(1:3).'; 1];
    aux.ee_kortex_urdf = kU(1:3) - R * F.toolOffset;
end
switch cfg.eeSource
    case 'fk'
        ee_pos = p;
        ee_vel = J(4:6, :) * dq;
    case 'kortex'
        if ~isempty(st)
            ee_pos = st.pose_real(1:3).';
            ee_vel = st.twist_real(1:3).';
        else                                  % Trockenlauf: Kortex-Werkzeugpunkt aus FK
            ee_pos = aux.tool_pred_real;
            w = J(1:3, :) * dq;
            ee_vel = Tinv(1:3, 1:3) * (J(4:6, :) * dq + cross(w, R * F.toolOffset));
        end
end
end

%% ========================================================================
%  LOG
%  ========================================================================
function L = init_log(n, nJ, nObs)
L = struct();
L.step_kind       = nan(n, 1);   % 0 normal, 1 OOD-Stopp, 2 anderer Stopp, 3 Konvergenz-Stopp
L.t_wall          = nan(n, 1);   % Schleifenstart [s] (schneller Trockenlauf: k*Ts)
L.t_ref           = nan(n, 1);
L.dt_loop         = nan(n, 1);
L.dt_since_send   = nan(n, 1);
L.timing_overrun  = false(n, 1);
L.dur_feedback    = nan(n, 1);
L.dur_fk          = nan(n, 1);
L.dur_obs         = nan(n, 1);
L.dur_agent       = nan(n, 1);
L.dur_pipeline    = nan(n, 1);
L.dur_send        = nan(n, 1);
L.dur_log         = nan(n, 1);
L.dur_wait        = nan(n, 1);
L.q               = nan(n, nJ);  % [rad], stetig abgewickelt
L.dq              = nan(n, nJ);
L.ee_pos          = nan(n, 3);   % Regelpunkt (fk: URDF-Frame, kortex: Kortex-Frame)
L.ee_vel          = nan(n, 3);
L.ee_ref          = nan(n, 3);   % Ziel im Frame des Regelpunkts
L.ee_fk           = nan(n, 3);   % FK-Endeffektor, URDF-Frame
L.tool_pred_real  = nan(n, 3);   % aus FK vorhergesagte tool_pose-Position, Kortex-Frame
L.ee_kortex_real  = nan(n, 3);   % gemessene tool_pose-Position, Kortex-Frame
L.ee_kortex_ori   = nan(n, 3);
L.twist_kortex    = nan(n, 6);
L.ee_kortex_urdf  = nan(n, 3);   % aus tool_pose zurueckgerechneter EE-Punkt, URDF-Frame
L.min_height      = nan(n, 1);
L.ep              = nan(n, 3);
L.ep_norm         = nan(n, 1);
L.ev              = nan(n, 3);
L.conv_flag       = false(n, 1);
L.conv_dwell      = nan(n, 1);
L.obs_raw         = nan(n, nObs);
L.obs             = nan(n, nObs);
L.dq_raw          = nan(n, nJ);
L.dq_sat          = nan(n, nJ);
L.dq_filt         = nan(n, nJ);
L.dq_rl           = nan(n, nJ);
L.dq_safe         = nan(n, nJ);
L.dq_cmd          = nan(n, nJ);
L.dq_sent_deg     = nan(n, nJ);
L.softlimit_active = false(n, nJ);
L.cap_active      = false(n, nJ);
end

function L = log_step(L, k, s)
fn = fieldnames(s);
for i = 1:numel(fn)
    if isfield(L, fn{i})
        if size(L.(fn{i}), 2) == 1
            L.(fn{i})(k) = s.(fn{i});
        else
            L.(fn{i})(k, :) = s.(fn{i});
        end
    end
end
end

function L = truncate_log(L, n)
fn = fieldnames(L);
for i = 1:numel(fn)
    L.(fn{i}) = L.(fn{i})(1:n, :);
end
end

function meta = build_meta(cfg, env, overrides, q_start, dev_deg, stopReason, stopStep, L)
meta = env;
meta.scriptName    = cfg.scriptName;
meta.scriptVersion = cfg.scriptVersion;
meta.condId        = cfg.condId;
meta.startId       = cfg.startId;
meta.repetition    = cfg.repetition;
meta.agentFile     = strrep(cfg.agentFile, [sk_path() filesep], '');
meta.agentLabel    = cfg.agentLabel;
meta.agentMd5      = cfg.agentMd5;
meta.agentSampleTime = cfg.agentSampleTime;
meta.pipelineTs    = cfg.pipelineTs;
meta.rateHz        = cfg.rateHz;
meta.maxDuration   = cfg.maxDuration;
meta.cmdScale      = cfg.cmdScale;
meta.eeSource      = cfg.eeSource;
meta.targetMode    = cfg.targetMode;
meta.target        = cfg.target(:).';
meta.startsMd5     = cfg.startsMd5;
meta.d0List_m      = cfg.d0List;
meta.oodThreshold  = cfg.oodThreshold;
meta.watchdogTimeout = cfg.watchdogTimeout;
meta.safetyFactor  = cfg.safetyFactor;
meta.dryRun        = cfg.dryRun;
meta.operatorNote  = cfg.operatorNote;
meta.overrides     = overrides;
meta.qStart_deg    = rad2deg(q_start(:)).';
meta.startDev_deg  = dev_deg;
meta.stopReason    = stopReason;
meta.stopStep      = stopStep;
meta.converged     = strcmp(stopReason, 'converged');
meta.nSteps        = size(L.t_wall, 1);
meta.tEnd          = L.t_wall(end);
ok = isfinite(L.ep_norm);
t = L.t_wall(ok); d = L.ep_norm(ok); P = L.ee_pos(ok, :);
k = set_point_kpis(t, d, P, 0.05, 0.5);
meta.d0_m = k.d0; meta.finalErr_m = k.finalErr; meta.minErr_m = k.minErr;
meta.success50 = k.success; meta.settle50_s = k.settle; meta.pathLen_m = k.pathLen;
meta.pathEff = k.pathEff; meta.overshoot_m = k.overshoot;
meta.rmsEp = sqrt(mean(d.^2)); meta.maxEp = max([d; NaN]);
dt = L.dt_loop(2:end); dt = dt(isfinite(dt));
meta.loopMean_ms   = 1e3 * mean(dt);
meta.loopMedian_ms = 1e3 * median(dt);
meta.loopRateMedian_Hz = 1 / median(dt);
sent = abs(L.dq_sent_deg); safe = rad2deg(abs(L.dq_safe));
m = safe > 1e-3;
meta.effFactorMeasured = median(sent(m) ./ safe(m));
res = vecnorm(L.ee_kortex_urdf - L.ee_fk, 2, 2);
meta.kortexFkResidualRms_m = sqrt(mean(res(isfinite(res)).^2));
meta.minHeight_m   = min(L.min_height);
meta.anyCapActive  = any(L.cap_active(:));
meta.anySoftLimit  = any(L.softlimit_active(:));
end

function k = set_point_kpis(t, d, P, tol, hold)
% Definitionen wie hardware/analysis/evaluate_p2p_hardware.m (Table VIII).
k.d0 = d(1); k.finalErr = d(end); k.minErr = min(d);
k.success = d(end) < tol;
k.settle = NaN;
idx = find(d < tol);
for i0 = idx(:).'
    msk = t >= t(i0) & t <= min(t(i0) + hold, t(end));
    if all(d(msk) < tol) && (t(end) - t(i0)) >= hold
        k.settle = t(i0) - t(1);
        break;
    end
end
k.pathLen = sum(vecnorm(diff(P), 2, 2));
k.pathEff = 0;
if k.pathLen > 1e-6, k.pathEff = min(1, k.d0 / k.pathLen); end
k.overshoot = 0;
if ~isempty(idx), k.overshoot = max(0, max(d(idx(1):end)) - tol); end
end

%% ========================================================================
%  HILFSFUNKTIONEN UND AUSGABE
%  ========================================================================
function q = unwrap_to(q, ref)
q = q + 2*pi * round((ref - q) / (2*pi));
end

function a = wrap_pi(a)
a = mod(a + pi, 2*pi) - pi;
end

function send_zero(apiHandle, cfg)
if ~cfg.dryRun && ~isempty(apiHandle)
    kinova_send_deg(apiHandle, zeros(1, cfg.nJ), cfg);
end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function print_safety_summary(cfg)
fprintf('\n== SICHERHEITSHINWEIS ==\n');
fprintf(' - Arbeitsraum frei, E-Stop bereit\n');
fprintf(' - Bedingung %s, Start %s (d0 = %.3f m), Wiederholung %d: %s\n', cfg.condId, cfg.startId, ...
        cfg.d0, cfg.repetition, cfg.purpose);
fprintf(' - Agent %s, Ts=%.3f s, Schleife %.0f Hz, hoechstens %.0f s\n', cfg.agentLabel, ...
        cfg.agentSampleTime, cfg.rateHz, cfg.maxDuration);
fprintf(' - Regelpunkt %s, Ziel [%s] m\n', cfg.eeSource, num2str(cfg.target(:).', '%.3f '));
fprintf(' - Befehlsfaktor cmdScale = %.2f (einziger Faktor)\n', cfg.cmdScale);
fprintf(' - HW-Cap %.2f * dqLim, OOD-Stopp bei %.2f m, Hoehenwaechter %.2f m, Watchdog %.2f s\n', ...
        cfg.safetyFactor, cfg.oodThreshold, cfg.heightGuard, cfg.watchdogTimeout);
bites = cfg.satUpper > 0 & cfg.dqHwCap < cfg.satUpper - 1e-6;
if any(bites)
    warning('deploy_setpoint_v24:cap', 'HW-Cap begrenzt Gelenke unter die Trainings-Saturation: J%s', ...
            mat2str(find(bites).'));
end
fprintf(' - Trockenlauf: %d\n\n', cfg.dryRun);
end

function plot_run(L, cfg, meta)
figure('Name', sprintf('%s %s r%02d', cfg.condId, cfg.startId, cfg.repetition));
subplot(1, 2, 1);
plot(L.t_wall, 1e3 * L.ep_norm, 'b-', 'LineWidth', 1.2); hold on;
yline(50, 'k--', '50 mm'); yline(1e3 * cfg.convDist, 'g--', 'Konvergenz');
xlabel('t [s]'); ylabel('||e_p|| [mm]'); grid on;
title(sprintf('%s: %s, Endfehler %.1f mm', cfg.startId, meta.stopReason, 1e3 * meta.finalErr_m), ...
      'Interpreter', 'none');
subplot(1, 2, 2);
plot3(L.ee_pos(:,1), L.ee_pos(:,2), L.ee_pos(:,3), 'b-', 'LineWidth', 1.2); hold on;
plot3(cfg.target(1), cfg.target(2), cfg.target(3), 'o', 'MarkerFaceColor', [0.92 0.41 0.20]);
plot3(L.ee_pos(1,1), L.ee_pos(1,2), L.ee_pos(1,3), 'ks');
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('Regelpunkt %s', cfg.eeSource));
end

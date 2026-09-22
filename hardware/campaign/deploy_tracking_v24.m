function result = deploy_tracking_v24(varargin)
%DEPLOY_TRACKING_V24  Tracking-Lauf auf dem Kinova Gen3 fuer die Messkampagne (V2.4).
%
%   result = DEPLOY_TRACKING_V24(condId, repetition)
%       Faehrt eine Bedingung aus campaign_plan (z. B. 'T10_nom', 1).
%   result = DEPLOY_TRACKING_V24(condId, repetition, Name, Value, ...)
%       Ueberschreibt einzelne Einstellungen, z. B. 'dryRun', true.
%   result = DEPLOY_TRACKING_V24(cfgStruct)
%       Freie Bedingung (condId 'custom'), nur fuer Tests.
%
%   Basis ist V2.3 (hardware/deploy/deploy_agent_kinova_robust_timing2.m) mit
%   derselben Observation [ep; ev; v_base; w_base; q; dq; e_ori], derselben
%   Trainings-Kette (Saturation -> IIR -> Rate Limiter) und derselben
%   Schutzschicht (Soft-Limit-Bremsung, HW-Cap, Fault-Check, OOD-Stopp,
%   Watchdog, onCleanup). Aenderungen gegenueber V2.3:
%     1. Kein versteckter Faktor mehr. Es gibt genau einen Befehlsfaktor
%        cfg.cmdScale, er steht in der Bedingung und im Log.
%     2. Der Rate Limiter nutzt die Abtastzeit des Agenten (cfg.pipelineTs),
%        nicht die Schleifen-Sollzeit. Damit ist die Kette pro Schritt
%        trainingsgleich, auch wenn Schleifenrate und Trainingsrate abweichen.
%     3. Auch der Schritt, der einen Stopp ausloest (OOD, Watchdog, Fault,
%        Sendefehler), wird geloggt. Grund steht in meta.stopReason.
%     4. Geloggt werden alle Teilzeiten (Feedback, FK, Observation, getAction,
%        Kette, Senden, Logging, Warten), die Observation vor und nach dem
%        Clipping, jede Stufe der Befehlskette und der exakt gesendete Befehl
%        in deg/s.
%     5. Die Kortex-Pose wird aus baseFb.tool_pose gelesen (wie im Set-Point-
%        Skript). In V2.3 blieb ee_kortex NaN.
%     6. Optionales Homing in die Trainingspose ueber ReachJointAngles
%        (Aufruf wie in playback_variants.m), danach Pruefung der Startabweichung.
%     7. Metadaten: Skriptversion, Git-Stand, Rechner, CPU, Agent-Datei mit
%        MD5, Abtastzeit des Agenten, alle Einstellungen. Strings als char,
%        damit die Logs auch mit scipy lesbar sind.
%     8. Keine Konsolenausgaben in der Schleife (sie verfaelschen das Timing).
%
%   Logs: data/hardware/campaign/<condId>/<condId>_rNN_<Zeitstempel>.mat
%   (Variable "run" mit run.meta, run.cfg, run.log) und eine Zeile in
%   data/hardware/campaign/campaign_index.csv. Trockenlaeufe landen in
%   data/hardware/campaign/_dryrun/.
%
%   SICHERHEIT: E-Stop in Reichweite, Arbeitsraum frei. Vor jedem echten Lauf
%   fragt das Skript nach ENTER. CTRL+C loest ueber onCleanup Zero-Velocity aus.

%% =========================
%  0) KONFIGURATION
%  =========================
cfg = default_config();
[cfg, overrides] = parse_inputs(cfg, varargin{:});
cfg = derive_config(cfg);

env = campaign_env_meta();
fprintf('\n=== deploy_tracking_v24 | %s r%02d | %s ===\n', cfg.condId, cfg.repetition, env.datetimeStart);
if env.gitDirty
    warning('deploy_tracking_v24:dirty', ['Git-Arbeitsstand hat uncommittete Aenderungen (%s). ' ...
        'Fuer Kampagnenlaeufe vorher committen, damit der Log einem Stand zugeordnet werden kann.'], env.gitHash);
end

%% =========================
%  1) AGENT LADEN + OBS-KONSISTENZ
%  =========================
assert(isfile(cfg.agentFile), 'Agent-Datei nicht gefunden: %s', cfg.agentFile);
fprintf('Lade Agent: %s\n', cfg.agentFile);
loaded = load(cfg.agentFile);
agent  = loaded.agent;
agent.UseExplorationPolicy = false;

obsInfo = getObservationInfo(agent);
assert(obsInfo.Dimension(1) == cfg.expectedObsDim, ...
    'ObservationDim Mismatch: Agent=%d, Deploy=%d.', obsInfo.Dimension(1), cfg.expectedObsDim);
cfg.agentSampleTime = agent.AgentOptions.SampleTime;
cfg.pipelineTs      = cfg.agentSampleTime;   % Rate Limiter wie im Training (Aenderung 2)
cfg.agentMd5        = file_md5(cfg.agentFile);
fprintf('Agent: Ts=%.3f s, MD5=%s, Schleifen-Soll %.1f Hz\n', cfg.agentSampleTime, cfg.agentMd5, cfg.rateHz);

%% =========================
%  2) ROBOTERMODELL + REFERENZ
%  =========================
assert(isfile(cfg.urdfFile), 'URDF nicht gefunden: %s', cfg.urdfFile);
robot_rbt = importrobot(cfg.urdfFile);
robot_rbt.DataFormat = 'row';

t_ref_full = linspace(0, cfg.maxDuration, max(2, ceil(cfg.maxDuration * 40) + 1));
traj_ref   = reference_trajectory(t_ref_full, cfg);

% Reihenfolge wie im Trainingsmodell (Mux "Observation"): [ep; ev; v_base; w_base; q; dq; e_ori]
obsLow  = [-cfg.ePLim*ones(3,1); -cfg.eVLim*ones(3,1); -cfg.vBLim*ones(3,1); -cfg.wBLim*ones(3,1); ...
            cfg.qLim_lower; -cfg.dqLim; -cfg.eOriLim*ones(3,1)];
obsHigh = [ cfg.ePLim*ones(3,1);  cfg.eVLim*ones(3,1);  cfg.vBLim*ones(3,1);  cfg.wBLim*ones(3,1); ...
            cfg.qLim_upper;  cfg.dqLim;  cfg.eOriLim*ones(3,1)];

if cfg.runPreflight
    preflight_reachability(robot_rbt, traj_ref, cfg);
end

%% =========================
%  3) MEX-VERBINDUNG, HOMING, STARTPRUEFUNG
%  =========================
apiHandle = [];
if ~cfg.dryRun
    fprintf('== MEX init ==\n');
    apiHandle = kinova_open(cfg);
    cleanupObj = onCleanup(@() kinova_safe_shutdown(apiHandle, cfg.nJ));
    st0 = kinova_read_state(apiHandle, cfg);
    if all(isnan(st0.pose_real(1:3)))
        warning('deploy_tracking_v24:toolpose', 'baseFb.tool_pose nicht lesbar, ee_kortex bleibt NaN.');
    end
    if cfg.doHoming
        fprintf('== Homing in die Trainingspose [%s] deg ==\n', num2str(rad2deg(cfg.qHome(:).'), '%g '));
        move_to_joints(apiHandle, cfg.qHome, cfg);
    end
    st0 = kinova_read_state(apiHandle, cfg);
    q_start = st0.q_rad;
else
    fprintf('\n=== TROCKENLAUF: keine MEX-Verbindung, Roboterzustand bleibt fest ===\n');
    q_start = cfg.qHome(:);
end

T_start   = getTransform(robot_rbt, q_start.', char(cfg.eeBodyName));
err_start = norm(T_start(1:3, 4) - traj_ref(1, :)');
fprintf('Startabweichung zur Referenz: %.4f m (Grenze %.3f m)\n', err_start, cfg.maxStartError);
if err_start > cfg.maxStartError && ~cfg.allowStartOffset
    error('deploy_tracking_v24:start', ['Startabweichung %.3f m > %.3f m. Roboter in die ' ...
        'Trainingspose bringen oder allowStartOffset setzen (wird geloggt).'], err_start, cfg.maxStartError);
end

print_safety_summary(cfg);
if ~cfg.dryRun
    input(sprintf('ENTER startet %s r%02d (CTRL+C bricht ab)... ', cfg.condId, cfg.repetition), 's');
end

%% =========================
%  4) WARM-UP (wie V2.3)
%  =========================
try
    getAction(agent, {zeros(cfg.expectedObsDim, 1)});
catch ME
    warning('deploy_tracking_v24:warmup', 'getAction warm-up fehlgeschlagen: %s', ME.message);
end
if ~cfg.dryRun
    for ii = 1:3
        stW = kinova_read_state(apiHandle, cfg);
        if stW.fault, error('deploy_tracking_v24:fault', 'Fault waehrend Warm-up -- Abbruch.'); end
        pause(0.02);
    end
end

%% =========================
%  5) HAUPTSCHLEIFE
%  =========================
state = struct('yFilt', zeros(cfg.nJ, 1), 'dqPrev', zeros(cfg.nJ, 1));
nStepsMax = ceil(cfg.maxDuration * max(cfg.rateHz, 10)) + ceil(2.0 * max(cfg.rateHz, 10)) + 10;
log = init_log(nStepsMax, cfg.nJ, cfg.expectedObsDim);

q  = q_start;
dq = zeros(cfg.nJ, 1);
stopReason = 'max_steps';
stopStep   = 0;
kLast      = 0;

r = rateControl(cfg.rateHz);
cfg.rateControlOverrunAction = char(r.OverrunAction);
try
    reset(r);
catch
    % aeltere MATLAB-Versionen starten rateControl beim Erzeugen
end

tRunStart = tic;
tRef = 0.0; tWallPrev = 0.0; tLastSend = NaN;

for k = 1:nStepsMax
    s = struct();
    t0 = toc(tRunStart);
    if k == 1, dtLoop = 0.0; else, dtLoop = t0 - tWallPrev; end
    tWallPrev = t0;
    if isnan(tLastSend), dtSinceSend = NaN; else, dtSinceSend = t0 - tLastSend; end
    s.t_wall = t0; s.dt_loop = dtLoop; s.dt_since_send = dtSinceSend;

    % --- Watchdog (wie V2.3: Zeit seit dem letzten Senden) ---
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

    % --- Referenzzeit ---
    switch cfg.referenceTiming
        case 'wall'
            tRef = min(t0, cfg.maxDuration);
        case 'sample'
            tRef = min((k - 1) * cfg.Ts, cfg.maxDuration);
        case 'guarded_wall'
            if k > 1
                tRef = min(tRef + min(max(dtLoop, 0), cfg.maxRefAdvanceFactor * cfg.Ts), cfg.maxDuration);
            end
        otherwise
            error('Unbekanntes referenceTiming: %s', cfg.referenceTiming);
    end
    if tRef >= cfg.maxDuration && k > 1
        send_zero(apiHandle, cfg);
        stopReason = 'completed'; stopStep = k - 1;
        break;
    end
    s.t_ref = tRef;

    % --- Feedback ---
    tA = toc(tRunStart);
    if ~cfg.dryRun
        st = kinova_read_state(apiHandle, cfg);
        q = st.q_rad; dq = st.dq_rad;
        s.ee_kortex_real = st.pose_real(1:3);
        s.ee_kortex_ori  = st.pose_real(4:6);
        s.twist_kortex   = st.twist_real;
        fault = st.fault;
    else
        fault = false;
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

    % --- FK + Jacobi ---
    T_ee   = getTransform(robot_rbt, q.', char(cfg.eeBodyName));
    J_ee   = geometricJacobian(robot_rbt, q.', char(cfg.eeBodyName));
    ee_pos = T_ee(1:3, 4);
    tC = toc(tRunStart);
    s.dur_fk = tC - tB;

    % --- Referenz, Fehler, Observation (Trainings-Konvention: Ist - Soll) ---
    [ref_pos_row, ref_vel_row] = reference_trajectory(tRef, cfg);
    ref_pos = ref_pos_row.'; ref_vel = ref_vel_row.';
    ee_vel  = J_ee(4:6, :) * dq;
    ep = ee_pos - ref_pos;
    ev = ee_vel - ref_vel;
    obs_raw = [ep; ev; zeros(3,1); zeros(3,1); q; dq; zeros(3,1)];
    obs = max(min(obs_raw, obsHigh), obsLow);
    tD = toc(tRunStart);
    s.dur_obs = tD - tC;
    s.ee_pos = ee_pos.'; s.ee_vel = ee_vel.'; s.ee_ref = ref_pos.'; s.ee_vref = ref_vel.';
    s.ep = ep.'; s.ep_norm = norm(ep); s.ev = ev.';
    s.obs_raw = obs_raw.'; s.obs = obs.';

    % --- OOD-Stopp (Schritt wird geloggt, Aenderung 3) ---
    if norm(ep) > cfg.oodThreshold
        send_zero(apiHandle, cfg);
        s.step_kind = 1;
        log = log_step(log, k, s); kLast = k;
        stopReason = 'ood'; stopStep = k;
        break;
    end

    % --- Agent ---
    action = getAction(agent, {obs});
    tE = toc(tRunStart);
    s.dur_agent = tE - tD;
    dq_raw = max(min(action{1}(:), 1), -1);

    % --- Trainings-Kette mit Zwischenstufen ---
    dq_sat = max(min(dq_raw, cfg.satUpper), cfg.satLower);
    state.yFilt = cfg.filt_num * dq_sat + cfg.filt_den * state.yFilt;
    maxDelta = cfg.slewRate * cfg.pipelineTs;
    dq_rl = state.dqPrev + max(min(state.yFilt - state.dqPrev, maxDelta), -maxDelta);
    state.dqPrev = dq_rl;

    % --- Schutzschicht (wie V2.3) ---
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

    % --- Einziger Befehlsfaktor (Aenderung 1) ---
    dq_cmd  = cfg.cmdScale * dq_safe;
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
log = truncate_log(log, max(kLast, 1));
meta = build_meta(cfg, env, overrides, q_start, err_start, stopReason, stopStep, log);

fprintf('\n== Lauf beendet: %s nach %d Schritten (%.2f s) ==\n', stopReason, meta.nSteps, meta.tEnd);
fprintf('   RMS ||ep|| = %.4f m, max = %.4f m, RMS erste 3.4 s = %.4f m\n', meta.rmsEp, meta.maxEp, meta.rmsEp3p4);
fprintf('   Schleife: Mittel %.1f ms, Median %.1f ms (%.2f Hz)\n', meta.loopMean_ms, meta.loopMedian_ms, meta.loopRateMedian_Hz);
fprintf('   Wirksamer Faktor (J2/J4, gemessen) = %.3f, konfiguriert %.3f\n', meta.effFactorMeasured, cfg.cmdScale);

run = struct();
run.schema = 'sk_campaign_v1';
run.meta = to_char_struct(meta);
run.cfg  = to_char_struct(cfg);
run.log  = log;

if cfg.dryRun
    saveDir = fullfile(cfg.saveRoot, '_dryrun', cfg.condId);
else
    saveDir = fullfile(cfg.saveRoot, cfg.condId);
end
if ~isfolder(saveDir), mkdir(saveDir); end
fname = sprintf('%s_r%02d_%s.mat', cfg.condId, cfg.repetition, ...
                char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
filePath = fullfile(saveDir, fname);
save(filePath, 'run', '-v7');
fprintf('Log gespeichert: %s\n', filePath);

idxRow = struct('file', fname, 'condId', cfg.condId, 'repetition', cfg.repetition, ...
    'datetime', env.datetimeStart, 'dryRun', cfg.dryRun, 'agent', cfg.agentLabel, ...
    'rateHz', cfg.rateHz, 'pathDuration', cfg.pathDuration, 'cmdScale', cfg.cmdScale, ...
    'stopReason', stopReason, 'nSteps', meta.nSteps, 'tEnd', meta.tEnd, 'rmsEp', meta.rmsEp, ...
    'maxEp', meta.maxEp, 'loopMean_ms', meta.loopMean_ms, 'gitHash', env.gitHash, ...
    'scriptVersion', cfg.scriptVersion);
append_index_csv(fullfile(cfg.saveRoot, ternary(cfg.dryRun, 'campaign_index_dryrun.csv', ...
    'campaign_index.csv')), idxRow);

if cfg.showPlots
    plot_run(log, traj_ref, cfg, meta);
end

result = struct('file', filePath, 'meta', meta);
end

%% ========================================================================
%  KONFIGURATION
%  ========================================================================
function cfg = default_config()
cfg = struct();
cfg.scriptName    = 'deploy_tracking_v24';
cfg.scriptVersion = 'V2.4';
cfg.condId        = 'custom';
cfg.repetition    = 0;
cfg.agentLabel    = '';
cfg.purpose       = '';
cfg.operatorNote  = '';

cfg.dryRun            = false;
cfg.rateControlInDryRun = true;
cfg.showPlots         = true;
cfg.runPreflight      = true;

cfg.agentFile      = '';
cfg.expectedObsDim = 29;

cfg.urdfFile   = sk_path('robot', 'SpaceKinova.urdf');
cfg.eeBodyName = 'kinova_end_effector_link';
cfg.toolOffset = [0; 0; 0.01];

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

cfg.safetyFactor = 0.75;             % HW-Cap = safetyFactor * dqLim (wie V2.3)
cfg.cmdScale     = 1.0;              % einziger Befehlsfaktor (Aenderung 1)

cfg.satUpper = [0.0; 0.9774; 0.0; 0.9774; 0.0; 0.1; 0.0];
cfg.satLower = -cfg.satUpper;
cfg.filt_num = 0.05;
cfg.filt_den = 0.95;
cfg.slewRate = 0.5;
cfg.qSoftMargin = deg2rad(10);

cfg.rateHz          = 10;            % Soll-Schleifenrate (rateControl)
cfg.pathDuration    = 8.5;           % Dauer des Halbkreises [s]
cfg.watchdogTimeout = [];            % Zeit seit letztem Senden [s], leer = max(0.10, 1.5*Ts).
                                     % V2.3 hatte fest 0.10 s. Bei 10 Hz liegt das genau auf
                                     % der Taktperiode und loest aus, sobald die Schleife
                                     % schneller als der Takt ist und auf ihn wartet.
cfg.referenceTiming = 'wall';
cfg.maxRefAdvanceFactor = 1.25;
cfg.stopOnTimingOverrun = true;

cfg.doHoming        = true;
cfg.qHome           = zeros(7, 1);   % Trainingspose (startpose_library.training) [rad]
cfg.homingTimeout_s = 60;
cfg.homingTol_deg   = 1.0;
cfg.homingSettle_s  = 2.0;
cfg.maxStartError   = 0.02;          % [m], sonst Abbruch
cfg.allowStartOffset = false;

cfg.r          = 0.2;
cfg.startPoint = [0.0, -0.025, 1.687];   % FK der Nullstellung, Start des Halbkreises
cfg.yConst     = 0;
cfg.realBaseOffset = [0 0 0.001];

cfg.ePLim = 0.5; cfg.eVLim = 1.0; cfg.vBLim = 0.5; cfg.wBLim = 1.0; cfg.eOriLim = pi;
cfg.oodThreshold = 0.4;

cfg.saveRoot = sk_path('data', 'hardware', 'campaign');
end

function [cfg, overrides] = parse_inputs(cfg, varargin)
overrides = struct();
if isempty(varargin)
    error('deploy_tracking_v24:args', 'Aufruf: deploy_tracking_v24(condId, repetition, ...)');
end
if isstruct(varargin{1})
    ov = varargin{1};
    rest = varargin(2:end);
else
    condId = char(varargin{1});
    assert(numel(varargin) >= 2 && isnumeric(varargin{2}), 'Wiederholung (Zahl) fehlt.');
    plan = campaign_plan();
    c = plan.tracking(strcmp({plan.tracking.condId}, condId));
    assert(~isempty(c), 'Bedingung %s nicht in campaign_plan.', condId);
    cfg.condId          = c.condId;
    cfg.repetition      = varargin{2};
    cfg.agentFile       = sk_path(c.agentFile);
    cfg.agentLabel      = c.agentLabel;
    cfg.rateHz          = c.rateHz;
    cfg.pathDuration    = c.pathDuration;
    cfg.cmdScale        = c.cmdScale;
    cfg.referenceTiming = c.referenceTiming;
    cfg.purpose         = c.purpose;
    ov = struct();
    rest = varargin(3:end);
end
assert(mod(numel(rest), 2) == 0, 'Name-Value-Paare erwartet.');
for i = 1:2:numel(rest)
    ov.(char(rest{i})) = rest{i + 1};
end
fn = fieldnames(ov);
for i = 1:numel(fn)
    assert(isfield(cfg, fn{i}), 'Unbekannte Einstellung: %s', fn{i});
    cfg.(fn{i}) = ov.(fn{i});
    overrides.(fn{i}) = ov.(fn{i});
end
cfg.referenceTiming = char(cfg.referenceTiming);
cfg.agentFile = char(cfg.agentFile);
end

function cfg = derive_config(cfg)
cfg.Ts          = 1 / cfg.rateHz;               % Soll-Schleifenperiode
cfg.maxDuration = cfg.pathDuration;
cfg.omega       = pi / cfg.maxDuration;
cfg.center      = [cfg.startPoint(1), cfg.startPoint(2), cfg.startPoint(3) - cfg.r];
cfg.dqHwCap     = cfg.safetyFactor * cfg.dqLim;
if isempty(cfg.watchdogTimeout)
    cfg.watchdogTimeout = max(0.10, 1.5 * cfg.Ts);
end
assert(cfg.cmdScale > 0 && cfg.cmdScale <= 1, 'cmdScale muss in (0, 1] liegen.');
assert(any(strcmp(cfg.referenceTiming, {'wall', 'sample', 'guarded_wall'})), 'referenceTiming unbekannt.');
end

%% ========================================================================
%  LOG
%  ========================================================================
function L = init_log(n, nJ, nObs)
L = struct();
L.step_kind       = nan(n, 1);   % 0 normal, 1 OOD-Stoppschritt, 2 anderer Stoppschritt
L.t_wall          = nan(n, 1);   % Schleifenstart seit tic [s]
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
L.q               = nan(n, nJ);  % [rad]
L.dq              = nan(n, nJ);  % [rad/s]
L.ee_pos          = nan(n, 3);   % FK, URDF-Frame [m]
L.ee_vel          = nan(n, 3);
L.ee_kortex_real  = nan(n, 3);   % Kortex tool_pose, reales Base-Frame [m]
L.ee_kortex_ori   = nan(n, 3);   % [deg]
L.twist_kortex    = nan(n, 6);
L.ee_ref          = nan(n, 3);
L.ee_vref         = nan(n, 3);
L.ep              = nan(n, 3);   % ee_pos - ee_ref
L.ep_norm         = nan(n, 1);
L.ev              = nan(n, 3);
L.obs_raw         = nan(n, nObs);
L.obs             = nan(n, nObs);
L.dq_raw          = nan(n, nJ);  % Agent-Ausgabe, auf [-1, 1] begrenzt
L.dq_sat          = nan(n, nJ);
L.dq_filt         = nan(n, nJ);
L.dq_rl           = nan(n, nJ);  % nach Rate Limiter
L.dq_safe         = nan(n, nJ);  % nach Soft-Limit und HW-Cap
L.dq_cmd          = nan(n, nJ);  % nach cmdScale [rad/s]
L.dq_sent_deg     = nan(n, nJ);  % exakt an die API uebergeben [deg/s]
L.softlimit_active = false(n, nJ);
L.cap_active      = false(n, nJ);
end

function L = log_step(L, k, s)
fn = fieldnames(s);
for i = 1:numel(fn)
    if isfield(L, fn{i})
        v = s.(fn{i});
        if size(L.(fn{i}), 2) == 1
            L.(fn{i})(k) = v;
        else
            L.(fn{i})(k, :) = v;
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

function meta = build_meta(cfg, env, overrides, q_start, err_start, stopReason, stopStep, L)
meta = env;
meta.scriptName    = cfg.scriptName;
meta.scriptVersion = cfg.scriptVersion;
meta.condId        = cfg.condId;
meta.repetition    = cfg.repetition;
meta.agentFile     = strrep(cfg.agentFile, [sk_path() filesep], '');
meta.agentLabel    = cfg.agentLabel;
meta.agentMd5      = cfg.agentMd5;
meta.agentSampleTime = cfg.agentSampleTime;
meta.pipelineTs    = cfg.pipelineTs;
meta.rateHz        = cfg.rateHz;
meta.pathDuration  = cfg.pathDuration;
meta.cmdScale      = cfg.cmdScale;
meta.referenceTiming = cfg.referenceTiming;
meta.oodThreshold  = cfg.oodThreshold;
meta.watchdogTimeout = cfg.watchdogTimeout;
meta.safetyFactor  = cfg.safetyFactor;
meta.dryRun        = cfg.dryRun;
meta.operatorNote  = cfg.operatorNote;
meta.overrides     = overrides;
meta.obsLayout     = {'ep_x','ep_y','ep_z','ev_x','ev_y','ev_z','vb_x','vb_y','vb_z', ...
                      'wb_x','wb_y','wb_z','q1','q2','q3','q4','q5','q6','q7', ...
                      'dq1','dq2','dq3','dq4','dq5','dq6','dq7','eori_x','eori_y','eori_z'};
meta.qStart_deg    = rad2deg(q_start(:)).';
meta.startError_m  = err_start;
meta.stopReason    = stopReason;
meta.stopStep      = stopStep;
meta.completed     = strcmp(stopReason, 'completed');
meta.nSteps        = size(L.t_wall, 1);
meta.tEnd          = L.t_wall(end);
e = L.ep_norm(isfinite(L.ep_norm));
meta.rmsEp = sqrt(mean(e.^2));
meta.maxEp = max([e; NaN]);
w = isfinite(L.ep_norm) & L.t_wall <= 3.4;
meta.rmsEp3p4 = sqrt(mean(L.ep_norm(w).^2));
dt = L.dt_loop(2:end); dt = dt(isfinite(dt));
meta.loopMean_ms   = 1e3 * mean(dt);
meta.loopMedian_ms = 1e3 * median(dt);
meta.loopRateMedian_Hz = 1 / median(dt);
% Wirksamer Faktor direkt am gesendeten Befehl: gesendet / Ausgang der Schutzschicht (J2, J4).
sent = abs(L.dq_sent_deg(:, [2 4])); safe = rad2deg(abs(L.dq_safe(:, [2 4])));
ok = safe > 1e-3;
meta.effFactorMeasured = median(sent(ok) ./ safe(ok));
% Definition wie bei den Altlaeufen (Table V): max|dq_cmd| / max|dq_filt|, J2 und J4.
% Enthaelt zusaetzlich die Wirkung des Rate Limiters und der Schutzschicht.
num = max(abs(L.dq_cmd(:, [2 4])), [], 1);
den = max(abs(L.dq_filt(:, [2 4])), [], 1);
meta.effFactorLegacy = mean(num ./ den, 'omitnan');
meta.anyCapActive  = any(L.cap_active(:));
meta.anySoftLimit  = any(L.softlimit_active(:));
end

%% ========================================================================
%  REFERENZ, PREFLIGHT, HOMING
%  ========================================================================
function [pos, vel] = reference_trajectory(t, cfg)
t = min(max(t(:), 0), cfg.maxDuration);
pos = [cfg.center(1) + cfg.r * sin(cfg.omega * t), ...
       cfg.center(2) + cfg.yConst * t, ...
       cfg.center(3) + cfg.r * cos(cfg.omega * t)];
vel = [cfg.r * cfg.omega * cos(cfg.omega * t), ...
       cfg.yConst * ones(size(t)), ...
      -cfg.r * cfg.omega * sin(cfg.omega * t)];
end

function preflight_reachability(robot_rbt, traj_ref, cfg)
fprintf('\n== Reachability-Preflight (wie V2.3) ==\n');
ik = inverseKinematics('RigidBodyTree', robot_rbt);
idx = round(linspace(1, size(traj_ref, 1), 20));
q_seed = zeros(1, cfg.nJ); nIk = 0; nLim = 0;
for i = idx
    [q_s, info] = ik(char(cfg.eeBodyName), trvec2tform(traj_ref(i, :)), [0.25 0.25 0.25 1 1 1], q_seed);
    if info.ExitFlag <= 0, nIk = nIk + 1; end
    q_w = mod(q_s(:) + pi, 2*pi) - pi;   % IK liefert teils um 2*pi versetzte Winkel
    if any(q_w < cfg.qLim_lower | q_w > cfg.qLim_upper), nLim = nLim + 1; end
    q_seed = q_s;
end
if nIk > 0 || nLim > 0
    warning('deploy_tracking_v24:preflight', ['Reachability-Preflight: IK ohne Konvergenz an %d von %d ' ...
        'Punkten, Gelenkgrenze verletzt an %d Punkten. Der Start liegt in der singulaeren ' ...
        'Nullstellung, dort konvergiert die IK oft nicht.'], nIk, numel(idx), nLim);
else
    fprintf('  OK\n');
end
end

function move_to_joints(apiHandle, q_target_rad, cfg)
% Aufruf wie playback_variants.m (dort auf Hardware erprobt).
target_deg = rad2deg(q_target_rad(:)).';
errCode = kortexApiMexInterface('ReachJointAngles', apiHandle, int32(0), 0, 0, target_deg);
if errCode ~= 0
    error('deploy_tracking_v24:homing', 'ReachJointAngles fehlgeschlagen (errorCode=%d).', errCode);
end
tStart = tic; stable = 0;
while toc(tStart) < cfg.homingTimeout_s
    st = kinova_read_state(apiHandle, cfg);
    if st.fault, error('deploy_tracking_v24:homingFault', 'Fault waehrend Homing.'); end
    err_deg = rad2deg(mod(st.q_rad - q_target_rad(:) + pi, 2*pi) - pi);
    if all(abs(err_deg) < cfg.homingTol_deg)
        stable = stable + 1;
        if stable >= 5, break; end
    else
        stable = 0;
    end
    pause(0.1);
end
if stable < 5
    error('deploy_tracking_v24:homingTimeout', 'Homing nicht innerhalb von %.0f s erreicht.', cfg.homingTimeout_s);
end
pause(cfg.homingSettle_s);
fprintf('  Homing OK\n');
end

function send_zero(apiHandle, cfg)
if ~cfg.dryRun && ~isempty(apiHandle)
    kinova_send_deg(apiHandle, zeros(1, cfg.nJ), cfg);
end
end

%% ========================================================================
%  AUSGABE
%  ========================================================================
function print_safety_summary(cfg)
fprintf('\n== SICHERHEITSHINWEIS ==\n');
fprintf(' - Arbeitsraum frei, E-Stop bereit\n');
fprintf(' - Bedingung %s, Wiederholung %d: %s\n', cfg.condId, cfg.repetition, cfg.purpose);
fprintf(' - Agent %s, Ts=%.3f s, Schleifen-Soll %.0f Hz, Bahn %.1f s\n', ...
        cfg.agentLabel, cfg.agentSampleTime, cfg.rateHz, cfg.pathDuration);
fprintf(' - Befehlsfaktor cmdScale = %.2f (einziger Faktor)\n', cfg.cmdScale);
fprintf(' - HW-Cap %.2f * dqLim, OOD-Stopp bei %.2f m, Watchdog %.2f s\n', ...
        cfg.safetyFactor, cfg.oodThreshold, cfg.watchdogTimeout);
bites = cfg.satUpper > 0 & cfg.dqHwCap < cfg.satUpper - 1e-6;
if any(bites)
    warning('deploy_tracking_v24:cap', 'HW-Cap begrenzt aktive Gelenke unter die Trainings-Saturation: J%s', ...
            mat2str(find(bites).'));
end
fprintf(' - Trockenlauf: %d\n\n', cfg.dryRun);
end

function plot_run(L, traj_ref, cfg, meta)
figure('Name', sprintf('%s r%02d: Bahn (xz)', cfg.condId, cfg.repetition));
plot(traj_ref(:,1), traj_ref(:,3), 'k:'); hold on;
plot(L.ee_ref(:,1), L.ee_ref(:,3), 'r--');
plot(L.ee_pos(:,1), L.ee_pos(:,3), 'b-');
if any(isfinite(L.ee_kortex_real(:)))
    ek = L.ee_kortex_real - cfg.realBaseOffset;
    plot(ek(:,1), ek(:,3), 'm-');
    legend('Plan', 'Referenz verwendet', 'FK', 'Kortex', 'Location', 'best');
else
    legend('Plan', 'Referenz verwendet', 'FK', 'Location', 'best');
end
axis equal; grid on; xlabel('x [m]'); ylabel('z [m]');
title(sprintf('%s: %s, RMS %.3f m', cfg.condId, meta.stopReason, meta.rmsEp), 'Interpreter', 'none');

figure('Name', sprintf('%s r%02d: Timing', cfg.condId, cfg.repetition));
D = 1e3 * [L.dur_feedback, L.dur_fk, L.dur_obs, L.dur_agent, L.dur_pipeline, L.dur_send, L.dur_log, L.dur_wait];
bar(L.t_wall, D, 'stacked');
legend('Feedback', 'FK', 'Obs', 'getAction', 'Kette', 'Senden', 'Log', 'Warten', 'Location', 'best');
xlabel('t_{wall} [s]'); ylabel('Dauer [ms]'); grid on;
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

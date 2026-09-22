function result = measure_loop_timing(condId, repetition, varargin)
%MEASURE_LOOP_TIMING  Loop-Zeit-Messung fuer Table III, Roboter steht (Nullbefehl).
%
%   result = MEASURE_LOOP_TIMING(condId, repetition)
%       condId aus campaign_plan().timing: 'M_send', 'M_fb', 'M_fk', 'M_full'.
%   result = MEASURE_LOOP_TIMING(condId, repetition, 'dryRun', true)
%       Ohne Roboter, nur zum Testen des Codepfads.
%
%   Die Schleife laeuft ohne rateControl so schnell wie moeglich und sendet in
%   jedem Zyklus SendJointSpeedCommand mit Null. Je nach Modus kommen hinzu:
%     send_only        : nur Senden
%     send_feedback    : + RefreshFeedback
%     send_feedback_fk : + FK und Jacobi (entspricht dem Open-Loop-Playback)
%     closed_loop_zero : + Observation, getAction, Trainings-Kette und Logging wie
%                        deploy_tracking_v24, gesendet wird trotzdem Null
%   Der Befehlsinhalt beeinflusst die API-Laufzeit nicht. Die Messung zeigt daher
%   dieselben Zeiten wie ein bewegter Lauf, ohne dass sich der Arm bewegt.
%
%   Logs: data/hardware/campaign/<condId>/<condId>_rNN_<Zeitstempel>.mat (Variable "run").

plan = campaign_plan();
c = plan.timing(strcmp({plan.timing.condId}, condId));
assert(~isempty(c), 'Timing-Bedingung %s nicht in campaign_plan.', condId);

cfg = struct();
cfg.scriptName = 'measure_loop_timing';
cfg.scriptVersion = 'V1.0';
cfg.condId = c.condId; cfg.repetition = repetition; cfg.mode = c.mode;
cfg.nCycles = c.nCycles; cfg.purpose = c.purpose;
cfg.agentFile = ''; if ~isempty(c.agentFile), cfg.agentFile = sk_path(c.agentFile); end
cfg.dryRun = false; cfg.operatorNote = '';
cfg.kinovaIP = '192.168.0.10'; cfg.kinovaUser = 'admin'; cfg.kinovaPassword = 'admin';
cfg.sessionTimeoutMs = uint32(60000); cfg.controlTimeoutMs = uint32(200); cfg.speedCmdDuration = 0;
cfg.nJ = 7;
cfg.urdfFile = sk_path('robot', 'SpaceKinova.urdf');
cfg.eeBodyName = 'kinova_end_effector_link';
cfg.satUpper = [0.0; 0.9774; 0.0; 0.9774; 0.0; 0.1; 0.0];
cfg.filt_num = 0.05; cfg.filt_den = 0.95; cfg.slewRate = 0.5;
cfg.saveRoot = sk_path('data', 'hardware', 'campaign');
for i = 1:2:numel(varargin)
    assert(isfield(cfg, varargin{i}), 'Unbekannte Einstellung: %s', varargin{i});
    cfg.(varargin{i}) = varargin{i + 1};
end
modes = {'send_only', 'send_feedback', 'send_feedback_fk', 'closed_loop_zero'};
iMode = find(strcmp(modes, cfg.mode));
assert(~isempty(iMode), 'Unbekannter Modus %s', cfg.mode);
env = campaign_env_meta();

needFk = iMode >= 3;
needAgent = iMode >= 4;
if needFk
    robot_rbt = importrobot(cfg.urdfFile); robot_rbt.DataFormat = 'row';
end
if needAgent
    loaded = load(cfg.agentFile); agent = loaded.agent; agent.UseExplorationPolicy = false;
    cfg.agentSampleTime = agent.AgentOptions.SampleTime;
    cfg.agentMd5 = file_md5(cfg.agentFile);
    getAction(agent, {zeros(29, 1)});   % Warm-up
end

apiHandle = [];
if ~cfg.dryRun
    apiHandle = kinova_open(cfg);
    cleanupObj = onCleanup(@() kinova_safe_shutdown(apiHandle, cfg.nJ));
    fprintf('\nRoboter muss stehen und frei sein. Es wird nur Null gesendet.\n');
    input(sprintf('ENTER startet %s r%02d (%s, %d Zyklen)... ', cfg.condId, repetition, cfg.mode, cfg.nCycles), 's');
end

n = cfg.nCycles;
L = struct('t_wall', nan(n,1), 'dt_loop', nan(n,1), 'dur_send', nan(n,1), 'dur_feedback', nan(n,1), ...
           'dur_fk', nan(n,1), 'dur_obs', nan(n,1), 'dur_agent', nan(n,1), 'dur_pipeline', nan(n,1), ...
           'dur_log', nan(n,1), 'fault', false(n,1));
Lx = struct('q', nan(n, 7), 'obs', nan(n, 29), 'dq_raw', nan(n, 7));   % Logging-Last wie im Deploy
q = zeros(cfg.nJ, 1); dq = zeros(cfg.nJ, 1);
yFilt = zeros(cfg.nJ, 1); dqPrev = zeros(cfg.nJ, 1);
tRun = tic; tPrev = 0;
for k = 1:n
    t0 = toc(tRun);
    L.t_wall(k) = t0;
    if k > 1, L.dt_loop(k) = t0 - tPrev; end
    tPrev = t0;
    if iMode >= 2
        tA = toc(tRun);
        if ~cfg.dryRun
            st = kinova_read_state(apiHandle, cfg);
            q = st.q_rad; dq = st.dq_rad; L.fault(k) = st.fault;
            if st.fault, warning('Fault in Zyklus %d, Messung beendet.', k); break; end
        end
        L.dur_feedback(k) = toc(tRun) - tA;
    end
    if needFk
        tB = toc(tRun);
        T_ee = getTransform(robot_rbt, q.', char(cfg.eeBodyName)); %#ok<NASGU>
        J_ee = geometricJacobian(robot_rbt, q.', char(cfg.eeBodyName));
        L.dur_fk(k) = toc(tRun) - tB;
    end
    if needAgent
        tC = toc(tRun);
        ee_vel = J_ee(4:6, :) * dq;
        obs = [zeros(3,1); ee_vel; zeros(6,1); q; dq; zeros(3,1)];
        L.dur_obs(k) = toc(tRun) - tC;
        tD = toc(tRun);
        a = getAction(agent, {obs});
        L.dur_agent(k) = toc(tRun) - tD;
        tE = toc(tRun);
        dq_raw = max(min(a{1}(:), 1), -1);
        yFilt = cfg.filt_num * max(min(dq_raw, cfg.satUpper), -cfg.satUpper) + cfg.filt_den * yFilt;
        md = cfg.slewRate * cfg.agentSampleTime;
        dqPrev = dqPrev + max(min(yFilt - dqPrev, md), -md);
        L.dur_pipeline(k) = toc(tRun) - tE;
    end
    tF = toc(tRun);
    if ~cfg.dryRun
        kinova_send_deg(apiHandle, zeros(1, cfg.nJ), cfg);
    end
    L.dur_send(k) = toc(tRun) - tF;
    if needAgent
        tG = toc(tRun);
        Lx.q(k, :) = q.'; Lx.obs(k, :) = obs.'; Lx.dq_raw(k, :) = dq_raw.';
        L.dur_log(k) = toc(tRun) - tG;
    end
end

dt = L.dt_loop(isfinite(L.dt_loop));
meta = env;
meta.scriptName = cfg.scriptName; meta.scriptVersion = cfg.scriptVersion;
meta.condId = cfg.condId; meta.repetition = repetition; meta.mode = cfg.mode;
meta.nCycles = n; meta.dryRun = cfg.dryRun; meta.operatorNote = cfg.operatorNote;
meta.loopMedian_ms = 1e3 * median(dt); meta.loopMean_ms = 1e3 * mean(dt);
meta.loopP95_ms = 1e3 * prctile(dt, 95); meta.loopRateMedian_Hz = 1 / median(dt);
fprintf('\n%s: Median %.1f ms (%.1f Hz), Mittel %.1f ms, p95 %.1f ms\n', cfg.mode, ...
        meta.loopMedian_ms, meta.loopRateMedian_Hz, meta.loopMean_ms, meta.loopP95_ms);

run = struct('schema', 'sk_campaign_timing_v1', 'meta', to_char_struct(meta), ...
             'cfg', to_char_struct(cfg), 'log', L);
if cfg.dryRun
    saveDir = fullfile(cfg.saveRoot, '_dryrun', cfg.condId);
else
    saveDir = fullfile(cfg.saveRoot, cfg.condId);
end
if ~isfolder(saveDir), mkdir(saveDir); end
fname = sprintf('%s_r%02d_%s.mat', cfg.condId, repetition, char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
save(fullfile(saveDir, fname), 'run', '-v7');
fprintf('Log gespeichert: %s\n', fullfile(saveDir, fname));
result = struct('file', fullfile(saveDir, fname), 'meta', meta);
end

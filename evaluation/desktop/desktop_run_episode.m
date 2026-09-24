function res = desktop_run_episode(cfg)
%DESKTOP_RUN_EPISODE  Simuliert eine Tracking-Episode und berechnet die Kennzahlen (Desktop-Plan D0).
%   res = desktop_run_episode(desktop_config('Ts_agent', 0.1))
%
%   res.metrics  Kennzahlen (Definitionen wie in evaluation/calculate_kpi_spacekinova.m):
%     mse_ee       mittleres Fehlerquadrat der EE-Position [m^2] (K1, ueber die Solver-Schritte)
%     rms_ee       Wurzel daraus [m]
%     max_ee       groesster EE-Fehler [m] (K2)
%     ori_mean     mittlerer Basis-Orientierungsfehler [rad] (K3), ori_max groesster
%     w_mean       mittlere Basis-Winkelgeschwindigkeit [rad/s] (K4)
%     ret          Return (K7)
%     early_stop   Episode vor T beendet (K8), t_end Endzeit, n_steps Agentenschritte
%     ood04        EE-Fehler ueberschreitet 0,4 m (OOD-Stopp der Hardware), ood04_step/_t erster Schritt
%     stop_reason  'none', 'ep>0.5m', 'ev>2m/s', 'ori>1rad', 'nonfinite' oder 'unknown'
%     sat_frac_J2/J4/J6   Anteil der Agentenschritte, in denen die Rohaktion die Saettigung
%                  ueberschreitet (A39)
%     qlim_frac    Anteil der Solver-Schritte mit einem Gelenk an der Positionssaettigung
%   res.ts       Zeitreihen (bei cfg.keepTs)
%   res.info     Laufzeit, Versionen, Git-Stand, Pruefsummen

if ~strcmp(cfg.model, 'SK_desktop') && ~bdIsLoaded(cfg.model)
    load_system(cfg.model);
end
if strcmp(cfg.model, 'SK_desktop')
    desktop_build_model();
    if ~bdIsLoaded('SK_desktop'), load_system('SK_desktop'); end
end

% --- Agent ---
agent = loadAgent(cfg.agentFile);
trainTs = agent.AgentOptions.SampleTime;
agent.AgentOptions.SampleTime = cfg.Ts_agent;
agent.UseExplorationPolicy = cfg.explore;

% --- Referenz ---
[EE_ref, EE_vref] = makeReference(cfg);

% --- Workspace ---
vars = struct('agent', agent, 'EE_ref', EE_ref, 'EE_vref', EE_vref, 'reward_init', 0, 'isdone_init', 0, ...
    'p_Ts', cfg.Ts, 'p_Ts_agent', cfg.Ts_agent, 'p_T', cfg.T, 'p_base_mass', cfg.base_mass, ...
    'p_delay_steps', cfg.delay_steps, 'p_damp_scale', cfg.damp_scale, 'p_slew', cfg.slew, ...
    'p_cmd_scale', cfg.cmd_scale, 'p_obs_mode', cfg.obs_mode);
fn = fieldnames(vars);
for k = 1:numel(fn)
    assignin('base', fn{k}, vars.(fn{k}));
end

in = Simulink.SimulationInput(cfg.model);
if ~strcmp(cfg.model, 'SK_desktop')
    in = in.setModelParameter('StopTime', num2str(cfg.T), 'FixedStep', num2str(cfg.Ts), ...
        'Solver', 'ode14x', 'SolverType', 'Fixed-step');
end

rng(cfg.seed, 'twister');
tWall = tic;
simOut = sim(in);
wallTime = toc(tWall);
if ~isempty(simOut.ErrorMessage)
    error('desktop_run_episode:sim', 'Simulation fehlgeschlagen: %s', simOut.ErrorMessage);
end

[metrics, ts] = computeMetrics(simOut.logsout, cfg);

res = struct();
res.cfg = cfg;
res.metrics = metrics;
if cfg.keepTs
    res.ts = ts;
else
    res.ts = struct();
end
res.info = struct('wallTime', wallTime, 'trainTs', trainTs, 'matlab', version, ...
    'git', gitInfo(), 'agentMD5', fileMD5(cfg.agentFile), ...
    'modelFile', get_param(cfg.model, 'FileName'), 'modelMD5', fileMD5(get_param(cfg.model, 'FileName')), ...
    'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
end

% =====================================================================
function agent = loadAgent(file)
persistent cache
if isempty(cache), cache = containers.Map(); end
if ~isKey(cache, file)
    S = load(file, 'agent');
    cache(file) = S.agent;
end
agent = cache(file);   % Handle-Objekt: SampleTime und UseExplorationPolicy werden je Lauf neu gesetzt
end

function [EE_ref, EE_vref] = makeReference(cfg)
% Halbkreis wie in calculate_kpi_spacekinova.m
t = 0:cfg.Ts:cfg.T;
omega = pi / cfg.T_path;
switch cfg.ref_timing
    case 'wall'
        x = cfg.center(1) + cfg.r * sin(omega * t);
        y = cfg.center(2) + 0 * t;
        z = cfg.center(3) + cfg.r * cos(omega * t);
        traj = [x(:), y(:), z(:)];
        vref = [zeros(1, 3); diff(traj) / mean(diff(t))];
    case 'sample'
        % Pro Agentenschritt rueckt die Referenzzeit um Ts_ref_step vor (V2.1). Zwischen den Schritten
        % bleibt sie stehen, die Beobachtung tastet ohnehin nur zu den Schrittzeitpunkten ab.
        tau = min(floor(t / cfg.Ts_agent + 1e-9) * cfg.Ts_ref_step, cfg.T_path);
        x = cfg.center(1) + cfg.r * sin(omega * tau);
        y = cfg.center(2) + 0 * tau;
        z = cfg.center(3) + cfg.r * cos(omega * tau);
        traj = [x(:), y(:), z(:)];
        vref = [cfg.r * omega * cos(omega * tau(:)), 0 * tau(:), -cfg.r * omega * sin(omega * tau(:))];
        vref(1, :) = 0;
    otherwise
        error('desktop_run_episode:ref', 'Unbekanntes ref_timing: %s', cfg.ref_timing);
end
EE_ref = timeseries(traj, t);
EE_vref = timeseries(vref, t);
end

function [m, ts] = computeMetrics(logsout, cfg)
m = struct();
ts = struct();

rew = getTs(logsout, 'reward');
t_agent = rew.Time(:);
m.ret = sum(rew.Data(:));
m.n_steps = numel(t_agent);
m.t_end = t_agent(end);

ee = getTs(logsout, 'EE_pos_differenz');
ep = toNx(ee.Data, 3);
epn = vecnorm(ep, 2, 2);
m.mse_ee = mean(epn.^2);
m.rms_ee = sqrt(m.mse_ee);
m.max_ee = max(epn);
m.nonfinite = any(~isfinite(epn));

ori = getTs(logsout, 'basis_orientation_error');
orin = vecnorm(toNx(ori.Data, 3), 2, 2);
m.ori_mean = mean(orin);
m.ori_max = max(orin);
wb = getTs(logsout, 'w_base');
m.w_mean = mean(vecnorm(toNx(wb.Data, 3), 2, 2));

% Basisbewegung (Position aus dem 6-DOF-Joint, falls geloggt)
qb = getTs(logsout, 'q_base');
m.base_disp_max = NaN;
if ~isempty(qb)
    X = toNx(qb.Data, []);
    if size(X, 2) == 3
        m.base_disp_max = max(vecnorm(X - X(1, :), 2, 2));
    end
end

% OOD-Schwelle der Hardware (0,4 m), ausgewertet zu den Agentenschritten
epn_agent = interp1(ee.Time(:), epn, t_agent, 'previous', 'extrap');
k04 = find(epn_agent > 0.4, 1);
m.ood04 = ~isempty(k04);
if m.ood04
    m.ood04_step = k04;
    m.ood04_t = t_agent(k04);
else
    m.ood04_step = NaN;
    m.ood04_t = NaN;
end

% Abbruch
isd = getTs(logsout, 'is_done');
if ~isempty(isd)
    m.early_stop = any(isd.Data(:) > 0.5) || m.t_end < cfg.T - 0.1;
else
    m.early_stop = m.t_end < cfg.T - 0.1;
end
m.stop_reason = 'none';
obs = getTs(logsout, 'obs');
if m.early_stop
    m.stop_reason = 'unknown';
    if ~isempty(obs)
        o = toNx(obs.Data, 29);
        o = o(end, :).';
        if any(~isfinite(o))
            m.stop_reason = 'nonfinite';
        elseif norm(o(1:3)) > 0.5
            m.stop_reason = 'ep>0.5m';
        elseif norm(o(4:6)) > 2.0
            m.stop_reason = 'ev>2m/s';
        elseif norm(o(27:29)) > 1.0
            m.stop_reason = 'ori>1rad';
        end
    elseif m.nonfinite
        m.stop_reason = 'nonfinite';
    elseif epn(end) > 0.5
        m.stop_reason = 'ep>0.5m';
    end
end

% Saettigung der Rohaktion (Grenzen aus dem Saturation-Block)
lim = [0.0; 0.9774; 0.0; 0.9774; 0.0; 0.1; 0.0];
araw = getTs(logsout, 'a_raw');
if ~isempty(araw)
    a = toNx(araw.Data, 7);
    for j = [2 4 6]
        m.(sprintf('sat_frac_J%d', j)) = mean(abs(a(:, j)) > lim(j) + 1e-12);
    end
    m.araw_absmax_J6 = max(abs(a(:, 6)));
else
    m.sat_frac_J2 = NaN; m.sat_frac_J4 = NaN; m.sat_frac_J6 = NaN; m.araw_absmax_J6 = NaN;
end

% Positionssaettigung hinter dem Integrator
qLim = [2*pi; 2.41; 2*pi; 2.66; 2.23; 2.01; 2*pi];
qc = getTs(logsout, 'q_cmd');
if ~isempty(qc)
    q = toNx(qc.Data, 7);
    atLim = abs(q) >= (qLim.' - 1e-9);
    m.qlim_frac = mean(any(atLim, 2));
else
    m.qlim_frac = NaN;
end

% Zeitreihen
ts.t_ee = ee.Time(:);
ts.ep = ep;
ts.t_agent = t_agent;
ts.reward = rew.Data(:);
names = {'a_raw', 'a_sat', 'a_filt', 'a_rl', 'a_scaled', 'obs', 'obs_agent'};
for k = 1:numel(names)
    s = getTs(logsout, names{k});
    if ~isempty(s)
        ts.(names{k}) = toNx(s.Data, []);
        ts.(['t_' names{k}]) = s.Time(:);
    end
end
if ~isempty(qc)
    ts.q_cmd = q;
    ts.t_q_cmd = qc.Time(:);
end
ts.ori = orin;
ts.t_ori = ori.Time(:);
end

function s = getTs(logsout, name)
% Erstes Element mit diesem Namen, leer wenn nicht geloggt
s = [];
names = logsout.getElementNames();
idx = find(strcmp(names, name), 1);
if isempty(idx), return; end
el = logsout.getElement(idx);
s = el.Values;
end

function X = toNx(raw, nCh)
% Zeitreihen-Daten in die Form N x Kanaele bringen
if ndims(raw) == 3
    X = reshape(permute(raw, [3 1 2]), size(raw, 3), []);
elseif isvector(raw)
    X = raw(:);
else
    X = raw;
    if ~isempty(nCh) && size(X, 2) ~= nCh && size(X, 1) == nCh
        X = X.';
    end
end
end

function g = gitInfo()
g = struct('hash', '', 'dirty', NaN);
[s1, h] = system(sprintf('git -C "%s" rev-parse --short HEAD', sk_path()));
[s2, d] = system(sprintf('git -C "%s" status --porcelain', sk_path()));
if s1 == 0, g.hash = strtrim(h); end
if s2 == 0, g.dirty = ~isempty(strtrim(d)); end
end

function h = fileMD5(file)
h = '';
try
    md = java.security.MessageDigest.getInstance('MD5');
    fid = fopen(file, 'r');
    data = fread(fid, inf, '*uint8');
    fclose(fid);
    md.update(data);
    h = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'))', 1, []));
catch
end
end

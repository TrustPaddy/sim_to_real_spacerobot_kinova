function res = desktop_run_setpoint(cfg)
%DESKTOP_RUN_SETPOINT  Simuliert eine Set-Point-Episode und berechnet die Kennzahlen (Desktop-Plan D4).
%   cfg-Felder:
%     model        'SK_point_fixed' (Basis fest, Schwerkraft, Rauschblock) oder 'SK_point' (frei schwebend)
%                  oder ein Originalmodell zum Vergleich
%     agentFile, agentLabel
%     q0_deg       Startpose [Grad], 7 Werte
%     target       Ziel im URDF-Frame [m], Standard [0,479, -0,005, 1,136]
%     T, Ts        Episodendauer (25 s) und Solver-Schritt (0,02 s)
%     base_mass    nur SK_point [kg]
%     noise_scale  nur SK_point_fixed: Faktor auf die Rauschstaerken aus SpaceKinova_Point_CDR.m (Phase 4 = 1)
%     explore, seed
%   Abbruch wie im Training: Erfolg, wenn EE-Abstand < 2 cm, EE-Geschwindigkeit < 3 cm/s und (frei schwebend)
%   Basisorientierung und -drehrate < 0,05 gleichzeitig gelten. Fehlschlag bei > 2 m.
%
%   res.metrics: final_err, min_err, t_end, converged (Erfolgsabbruch), failed (Fehlschlagabbruch),
%     success50 (Endfehler < 50 mm wie Table VIII), settle_t (ab hier dauerhaft < 50 mm),
%     frac_softzone_J2/J4/J6 (Anteil der Zeit naeher als 10 Grad an der Trainingsgrenze),
%     frac_atlim (ein Gelenk an der Trainingsgrenze), frac_beyond_hw (ein Gelenk jenseits der Gen3-Grenze:
%     J2 128,9, J4 147,8, J6 120,3 Grad), ori_max, ret

d = struct('model', 'SK_point_fixed', 'agentFile', '', 'agentLabel', '', 'q0_deg', zeros(1, 7), ...
    'target', [0.479, -0.005, 1.136], 'T', 25, 'Ts', 0.02, 'base_mass', 65, 'noise_scale', 0, ...
    'explore', false, 'seed', 0, 'label', '', 'keepTs', true);
fn = fieldnames(d);
for k = 1:numel(fn)
    if ~isfield(cfg, fn{k}), cfg.(fn{k}) = d.(fn{k}); end
end

desktop_build_point_models();
if ~bdIsLoaded(cfg.model), load_system(cfg.model); end

persistent cache
if isempty(cache), cache = containers.Map(); end
if ~isKey(cache, cfg.agentFile)
    S = load(cfg.agentFile, 'agent');
    cache(cfg.agentFile) = S.agent;
end
agent = cache(cfg.agentFile);
agent.UseExplorationPolicy = cfg.explore;

sigma = [0.003 * ones(3,1); 0.010 * ones(3,1); 0.005 * ones(3,1); 0.005 * ones(3,1); ...
         0.002 * ones(7,1); 0.010 * ones(7,1); 0.003 * ones(3,1)];

t = (0:cfg.Ts:cfg.T)';
EE_ref = timeseries(repmat(cfg.target(:).', numel(t), 1), t);
EE_vref = timeseries(zeros(numel(t), 3), t);
q0 = deg2rad(cfg.q0_deg(:));
vars = struct('agent', agent, 'EE_ref', EE_ref, 'EE_vref', EE_vref, 'reward_init', 0, 'isdone_init', 0, ...
    'q0', q0, 'dq0', zeros(7, 1), 'q_target', q0, 'q_des', repmat(q0.', numel(t), 1), ...
    'obs_noise_sigma', cfg.noise_scale * sigma, 'p_base_mass', cfg.base_mass);
fn = fieldnames(vars);
for k = 1:numel(fn)
    assignin('base', fn{k}, vars.(fn{k}));
end

in = Simulink.SimulationInput(cfg.model);
in = in.setModelParameter('StopTime', num2str(cfg.T), 'FixedStep', num2str(cfg.Ts), ...
    'Solver', 'ode14x', 'SolverType', 'Fixed-step');
rng(cfg.seed, 'twister');
tw = tic;
simOut = sim(in);
wallTime = toc(tw);
if ~isempty(simOut.ErrorMessage)
    error('desktop_run_setpoint:sim', 'Simulation fehlgeschlagen: %s', simOut.ErrorMessage);
end
L = simOut.logsout;

m = struct();
ee = getTs(L, 'EE_pos_differenz');
ep = toNx(ee.Data, 3);
epn = vecnorm(ep, 2, 2);
te = ee.Time(:);
m.final_err = epn(end);
m.min_err = min(epn);
m.t_end = te(end);
early = te(end) < cfg.T - 1e-6;
m.converged = early && epn(end) < 0.05;
m.failed = early && ~m.converged;
m.success50 = epn(end) < 0.05;
kOut = find(epn >= 0.05, 1, 'last');
if isempty(kOut)
    m.settle_t = 0;
elseif kOut == numel(epn)
    m.settle_t = NaN;
else
    m.settle_t = te(kOut + 1);
end
rew = getTs(L, 'reward');
m.ret = sum(rew.Data(:));
ori = getTs(L, 'basis_orientation_error');
if ~isempty(ori)
    m.ori_max = max(vecnorm(toNx(ori.Data, 3), 2, 2));
else
    m.ori_max = 0;
end

qc = getTs(L, 'q_cmd');
if isempty(qc), qc = getTs(L, 'q'); end   % Originalmodelle loggen nur q
q = toNx(qc.Data, 7);
limTrain = [2*pi; 2.41; 2*pi; 2.66; 2.23; 2.01; 2*pi].';
limHw = deg2rad([inf; 128.9; inf; 147.8; inf; 120.3; inf]).';
soft = deg2rad(10);
for j = [2 4 6]
    m.(sprintf('frac_softzone_J%d', j)) = mean(abs(q(:, j)) > limTrain(j) - soft);
    m.(sprintf('qmax_deg_J%d', j)) = rad2deg(max(abs(q(:, j))));
end
m.frac_atlim = mean(any(abs(q) >= limTrain - 1e-9, 2));
m.frac_beyond_hw = mean(any(abs(q) > limHw, 2));
m.d0 = norm(ep(1, :));

res = struct('cfg', cfg, 'metrics', m, 'info', struct('wallTime', wallTime, 'matlab', version, ...
    'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'))));
if cfg.keepTs
    res.ts = struct('t', te, 'epn', epn, 't_q', qc.Time(:), 'q', q);
else
    res.ts = struct();
end
end

function s = getTs(logsout, name)
s = [];
names = logsout.getElementNames();
idx = find(strcmp(names, name), 1);
if isempty(idx), return; end
s = logsout.getElement(idx).Values;
end

function X = toNx(raw, nCh)
if ndims(raw) == 3
    X = reshape(permute(raw, [3 1 2]), size(raw, 3), []);
elseif isvector(raw)
    X = raw(:);
else
    X = raw;
    if size(X, 2) ~= nCh && size(X, 1) == nCh, X = X.'; end
end
end

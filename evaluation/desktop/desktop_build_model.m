function mdlFile = desktop_build_model(force)
%DESKTOP_BUILD_MODEL  Erzeugt das parametrisierte Auswertemodell SK_desktop (Desktop-Plan D0).
%   mdlFile = desktop_build_model()       baut das Modell nur, wenn es fehlt
%   mdlFile = desktop_build_model(true)   baut es neu
%
%   Quelle ist models/SpaceKinova_MotionProfile_CDR.slx. Es entspricht dem 40-Hz-Modell mit einem
%   zusaetzlichen Delay-Block hinter dem Agenten (Befund A23). Die Quelle bleibt unveraendert.
%   Im neuen Modell sind diese Groessen Variablen (Werte setzt desktop_run_episode):
%     p_Ts           Solver-Schritt [s] (Rate Transition vor dem Integrator)
%     p_Ts_agent     Agenten-Takt [s] (Beobachtung, Reward)
%     p_T            Episodendauer [s]
%     p_base_mass    Basismasse [kg], Traegheit skaliert mit (Wuerfel 1 m). 1e9 kg haelt die Basis fest
%     p_delay_steps  Aktionsverzoegerung in Agentenschritten (0 = keine)
%     p_damp_scale   Faktor auf die Gelenkdaempfung (ohne Wirkung auf die Bewegung, Gelenke sind
%                    bewegungsgesteuert, A23)
%     p_slew         Anstieg des Rate Limiters [1/s] (Training 0,5)
%     p_cmd_scale    Faktor auf den Befehl nach der Kette (speedScale der Deploy-Skripte, A18)
%     p_obs_mode     0 = Beobachtung wie im Training, 1 = wie Deploy-Skript V2.1 (A22)
%     p_obs_noise    29x1 Standardabweichungen fuer Beobachtungsrauschen vor dem Agenten (D7), Standard 0
%   Zusaetzlich geloggt: Rohaktion, gesaettigte, gefilterte, ratenbegrenzte und skalierte Aktion,
%   Gelenkwinkel-Befehl hinter der Positionssaettigung, Beobachtung (obs) und Agenten-Eingang (obs_agent),
%   isDone.

if nargin < 1, force = false; end

src = 'SpaceKinova_MotionProfile_CDR';
dst = 'SK_desktop';
outDir = sk_path('evaluation', 'desktop', 'models');
mdlFile = fullfile(outDir, [dst '.slx']);
if ~isfolder(outDir), mkdir(outDir); end
addpath(outDir);
if isfile(mdlFile) && ~force
    return
end
if bdIsLoaded(dst), close_system(dst, 0); end

load_system(src);
srcFile = get_param(src, 'FileName');
copyfile(srcFile, mdlFile, 'f');
fileattrib(mdlFile, '+w');
close_system(src, 0);
load_system(mdlFile);

% --- Solver und Dauer ---
set_param(dst, 'FixedStep', 'p_Ts', 'StopTime', 'p_T');

% --- Raten ---
set_param([dst '/Rate Transition'], 'OutPortSampleTime', 'p_Ts');
set_param([dst '/Rate Transition1'], 'OutPortSampleTime', 'p_Ts_agent');
for k = 1:4
    set_param(sprintf('%s/Reward/Rate Transition%d', dst, k), 'OutPortSampleTime', 'p_Ts_agent');
end
% Der MATLAB-Function-Block des Rewards hat im Original eine feste Abtastzeit von 0,025 s
set_param([dst '/Reward/MATLAB Function'], 'SystemSampleTime', 'p_Ts_agent');

% --- Basis ---
inertia = [dst '/Robot/base_link/Inertia'];
set_param(inertia, 'Mass', 'p_base_mass', ...
    'MomentsOfInertia', '[10.833, 10.833, 10.833] * p_base_mass / 65');

% --- Verzoegerung ---
set_param([dst '/Delay'], 'DelayLength', 'p_delay_steps', 'DelayLengthUpperLimit', '100');

% --- Daempfung (Nennwerte wie im 40-Hz-Modell: J1-J4 0,5, J5-J7 0,3) ---
for j = 1:7
    blk = sprintf('%s/Robot/kinova_kinova_joint_%d', dst, j);
    if j <= 4, d0 = '0.5'; else, d0 = '0.3'; end
    set_param(blk, 'DampingCoefficient', [d0 ' * p_damp_scale']);
end

% --- Logging der Befehlskette ---
logPort([dst '/RL_Agent'], 1, 'a_raw');
logPort([dst '/Saturation'], 1, 'a_sat');
logPort([dst '/Discrete Filter'], 1, 'a_filt');
logPort([dst '/Rate Limiter'], 1, 'a_rl');
logPort([dst '/Saturation1'], 1, 'q_cmd');
logPort([dst '/Rate Transition1'], 1, 'obs');
logPort([dst '/Reward'], 2, 'is_done');

% --- Rate Limiter: Anstieg als Variable [1/s] ---
set_param([dst '/Rate Limiter'], 'RisingSlewLimit', 'p_slew', 'FallingSlewLimit', '-p_slew');

% --- Befehlsskalierung nach der Kette (wie speedScale der Deploy-Skripte, A18) ---
delete_line(dst, 'Rate Limiter/1', 'Rate Transition/1');
add_block('simulink/Math Operations/Gain', [dst '/Cmd Scale'], 'Gain', 'p_cmd_scale', ...
    'Position', blockPos([dst '/Rate Limiter'], [60 0]));
add_line(dst, 'Rate Limiter/1', 'Cmd Scale/1', 'autorouting', 'on');
add_line(dst, 'Cmd Scale/1', 'Rate Transition/1', 'autorouting', 'on');

% --- Beobachtung vor dem Agenten: korrekt (0) oder wie Deploy-Skript V2.1 (1, Befund A22) ---
delete_line(dst, 'Rate Transition1/1', 'RL_Agent/1');
obsBlk = [dst '/Obs Transform'];
add_block('simulink/User-Defined Functions/MATLAB Function', obsBlk, ...
    'Position', blockPos([dst '/Rate Transition1'], [80 0]));
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', obsBlk);
chart.Script = obsTransformCode();
add_block('simulink/Sources/Constant', [dst '/Obs Mode'], 'Value', 'p_obs_mode', ...
    'Position', blockPos([dst '/Rate Transition1'], [0 60]));
add_block('simulink/Sources/Constant', [dst '/Obs Noise'], 'Value', 'p_obs_noise', ...
    'Position', blockPos([dst '/Rate Transition1'], [0 120]));
add_line(dst, 'Rate Transition1/1', 'Obs Transform/1', 'autorouting', 'on');
add_line(dst, 'Obs Mode/1', 'Obs Transform/2', 'autorouting', 'on');
add_line(dst, 'Obs Noise/1', 'Obs Transform/3', 'autorouting', 'on');
add_line(dst, 'Obs Transform/1', 'RL_Agent/1', 'autorouting', 'on');
logPort(obsBlk, 1, 'obs_agent');
logPort([dst '/Cmd Scale'], 1, 'a_scaled');

set_param(dst, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
save_system(dst, mdlFile);
close_system(dst, 0);
fprintf('Modell erzeugt: %s\n', mdlFile);
end

function p = blockPos(ref, offset)
% Position relativ zu einem vorhandenen Block (nur fuer die Darstellung)
r = get_param(ref, 'Position');
p = [r(1) + offset(1), r(2) + offset(2) + 40, r(1) + offset(1) + 50, r(2) + offset(2) + 70];
end

function code = obsTransformCode()
code = strjoin({
'function y = obs_transform(u, mode, sigma)'
'% mode 0: Beobachtung wie im Training [ep; ev; v_base; w_base; q; dq; e_ori]'
'% mode 1: wie Deploy-Skript V2.1 (Befund A22): Reihenfolge [ep; ev; q; dq; v_base; w_base; e_ori],'
'%         Vorzeichen Soll - Ist, Basisgroessen null, e_ori = Endeffektor-Orientierung, Clipping wie V2.1'
'% sigma: 29x1 Standardabweichungen fuer gaussches Beobachtungsrauschen (D7), 0 = kein Rauschen.'
'%        Das Rauschen kommt aus MATLAB (desktop_obs_randn), damit rng-Seeds wirken.'
'coder.extrinsic(''desktop_v21_eori'', ''desktop_obs_randn'');'
'y = u;'
'if any(sigma > 0)'
'    nz = zeros(29, 1);'
'    nz = desktop_obs_randn();'
'    y = y + sigma .* nz;'
'end'
'if mode == 1'
'    q = y(13:19);'
'    dq = y(20:26);'
'    e = zeros(3, 1);'
'    e = desktop_v21_eori(q);'
'    y = [-y(1:3); -y(4:6); q; dq; zeros(3, 1); zeros(3, 1); e];'
'    qlo = [-2*pi; -2.41; -2*pi; -2.66; -2.23; -2.01; -2*pi];'
'    dqlim = [1.3963; 1.3963; 1.3963; 1.3963; 1.2218; 1.2218; 1.2218];'
'    hi = [0.5*ones(3,1); 1.0*ones(3,1); -qlo; dqlim; 0.5*ones(3,1); 1.0*ones(3,1); pi*ones(3,1)];'
'    y = min(max(y, -hi), hi);'
'end'
}, newline);
end

function logPort(blk, portNum, name)
ph = get_param(blk, 'PortHandles');
p = ph.Outport(portNum);
set_param(p, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

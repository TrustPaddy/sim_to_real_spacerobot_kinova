function desktop_build_point_models(force)
%DESKTOP_BUILD_POINT_MODELS  Erzeugt die Set-Point-Auswertemodelle (Desktop-Plan D4).
%   desktop_build_point_models()       baut fehlende Modelle
%   desktop_build_point_models(true)   baut beide neu
%
%   SK_point_fixed  Kopie von models/SpaceKinova_MotionProfile_point_fixed.slx (Trainingsmodell von
%                   test_agent_fixed1: Basis fest verschweisst, Schwerkraft an, Basisbeobachtungen null,
%                   Rauschblock mit obs_noise_sigma, Startpose q0). Zusaetzliches Logging, Rauschblock im
%                   Agenten-Takt 0,1 s (im Original geerbt, damit laeuft es mit sim() nicht).
%   SK_point        Kopie von models/SpaceKinova_MotionProfile_point.slx (frei schwebend, Schwerkraft aus).
%                   Basismasse (p_base_mass, Traegheit skaliert mit) und Startpose (q0) als Variablen, im
%                   Original fest 650 kg und [0 -90 0 0 0 0 0] Grad.
%   Geloggt werden zusaetzlich die Rohaktion (a_raw), der Gelenkwinkel-Befehl hinter der Positionssaettigung
%   (q_cmd) und isDone. Die Originale bleiben unveraendert.

if nargin < 1, force = false; end
outDir = sk_path('evaluation', 'desktop', 'models');
if ~isfolder(outDir), mkdir(outDir); end
addpath(outDir);

pairs = {'SpaceKinova_MotionProfile_point_fixed', 'SK_point_fixed'; ...
         'SpaceKinova_MotionProfile_point', 'SK_point'};
for p = 1:size(pairs, 1)
    src = pairs{p, 1}; dst = pairs{p, 2};
    mdlFile = fullfile(outDir, [dst '.slx']);
    if isfile(mdlFile) && ~force, continue; end
    if bdIsLoaded(dst), close_system(dst, 0); end
    load_system(src);
    copyfile(get_param(src, 'FileName'), mdlFile, 'f');
    fileattrib(mdlFile, '+w');
    close_system(src, 0);
    load_system(mdlFile);

    if strcmp(dst, 'SK_point_fixed')
        % Der Rauschblock (randn) erbt im Original eine kontinuierliche Abtastzeit und laeuft so mit sim()
        % nicht. Der Agent tastet die Beobachtung mit 0,1 s ab, deshalb Rauschen im Agenten-Takt.
        set_param([dst '/MATLAB Function'], 'SystemSampleTime', '0.1');
    end
    if strcmp(dst, 'SK_point')
        set_param([dst '/Robot/base_link/Inertia'], 'Mass', 'p_base_mass', ...
            'MomentsOfInertia', '[10.833, 10.833, 10.833] * p_base_mass / 65');
        set_param([dst '/Integrator'], 'InitialCondition', 'q0');
    end
    logPort([dst '/RL_Agent'], 1, 'a_raw');
    logPort([dst '/Saturation1'], 1, 'q_cmd');
    logPort([dst '/Reward'], 2, 'is_done');
    set_param(dst, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');
    save_system(dst, mdlFile);
    close_system(dst, 0);
    fprintf('Modell erzeugt: %s\n', mdlFile);
end
end

function logPort(blk, portNum, name)
ph = get_param(blk, 'PortHandles');
set_param(ph.Outport(portNum), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

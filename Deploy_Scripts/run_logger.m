function filepath = run_logger(data, meta, varargin)
%RUN_LOGGER Speichert einen Experiment-Run mit automatischer Nummerierung.
%
%   Alle Experiment-Skripte (Kinematik-Playback, Sim-Eval, Agent-Deployment)
%   rufen diese Funktion am Ende auf, damit jeder Lauf eindeutig
%   durchnummeriert und mit Metadaten versehen in runs/ landet.
%
%   USAGE:
%       filepath = run_logger(data, meta)
%       filepath = run_logger(data, meta, 'runDir', 'runs')
%       filepath = run_logger(data, meta, 'prefix', 'sim')
%       filepath = run_logger(data, meta, 'dryRun', true)
%
%   INPUTS:
%       data : struct mit allen Messreihen, z.B.
%           data.t             Nx1 Zeitvektor [s]
%           data.q_measured    Nx7 gemessene Gelenke [deg]
%           data.dq_cmd        Nx7 Kommando-Geschwindigkeiten [deg/s]
%           data.ee_measured   Nx3 EE-Position via FK [m]
%           data.obs           NxD Observation der Policy (wenn vorhanden)
%           data.action        NxA Policy-Action (wenn vorhanden)
%
%       meta : struct mit Metadaten:
%           meta.label         (erforderlich) kurzer Dateiname-Tag,
%                              z.B. 'kin_baseline_zero', 'agent_elbow_seed3'
%           meta.startpose     (empfohlen) 1x7 Startwinkel [deg]
%           meta.source        (empfohlen) 'real' | 'sim'
%           meta.seed          (optional) RNG-Seed
%           meta.checkpoint    (optional) Pfad/Name des Policy-Checkpoints
%           meta.urdfFile      (optional) verwendete URDF-Datei
%           meta.comment       (optional) Freitext
%
%   OUTPUT:
%       filepath : vollstaendiger Pfad der gespeicherten .mat-Datei
%
%   AUTOMATISCH HINZUGEFUEGT zu meta:
%       meta.datetime, meta.matlabVersion, meta.hostname,
%       meta.runNumber, meta.gitHash, meta.scriptCaller
%
%   BEISPIEL:
%       data = struct('t', t, 'q_measured', qlog, 'dq_cmd', dqlog);
%       meta = struct('label', 'kin_baseline_zero', ...
%                     'source', 'real', ...
%                     'startpose', [0 0 0 0 0 0 0], ...
%                     'comment', 'Erster Baseline-Lauf mit Zero-Pose');
%       fp = run_logger(data, meta);

    % -------- Optionale Argumente parsen --------
    p = inputParser;
    addParameter(p, 'runDir', 'runs', @(x) ischar(x) || isstring(x));
    addParameter(p, 'prefix', 'run',  @(x) ischar(x) || isstring(x));
    addParameter(p, 'dryRun', false);
    parse(p, varargin{:});
    runDir = char(p.Results.runDir);
    prefix = char(p.Results.prefix);
    dryRun = logical(p.Results.dryRun);

    % -------- Input-Validierung --------
    if ~isstruct(data)
        error('run_logger:badInput', 'data muss ein struct sein.');
    end
    if ~isstruct(meta)
        error('run_logger:badInput', 'meta muss ein struct sein.');
    end
    if ~isfield(meta, 'label') || isempty(meta.label)
        error('run_logger:missingLabel', ...
            'meta.label ist erforderlich (z.B. "kin_baseline_zero").');
    end

    label = sanitizeLabel(meta.label);

    recommended = {'startpose', 'source'};
    for i = 1:numel(recommended)
        f = recommended{i};
        if ~isfield(meta, f)
            warning('run_logger:missingMeta', ...
                'meta.%s fehlt. Fuer spaetere Analyse gut einzubauen.', f);
        end
    end

    % -------- Ordner anlegen --------
    if ~isfolder(runDir)
        mkdir(runDir);
        fprintf('[run_logger] Ordner angelegt: %s\n', runDir);
    end

    % -------- Laufnummer bestimmen --------
    runNumber = nextRunNumber(runDir, prefix);
    filename  = sprintf('%s_%03d_%s.mat', prefix, runNumber, label);
    filepath  = fullfile(runDir, filename);

    % -------- Automatische Metadaten ergaenzen --------
    meta.datetime      = datestr(now, 'yyyy-mm-dd HH:MM:SS');
    meta.matlabVersion = version;
    meta.hostname      = getHostname();
    meta.runNumber     = runNumber;
    meta.gitHash       = tryGetGitHash();
    meta.scriptCaller  = getCallerName();

    % -------- Speichern --------
    if dryRun
        fprintf('[run_logger] DRY RUN -- wuerde speichern: %s\n', filepath);
        return;
    end

    try
        save(filepath, 'data', 'meta', '-v7');
    catch ME
        warning('run_logger:saveFallback', ...
            'Speichern mit -v7 fehlgeschlagen (%s), versuche -v7.3.', ...
            ME.message);
        save(filepath, 'data', 'meta', '-v7.3');
    end

    fprintf('[run_logger] Run #%d gespeichert: %s\n', runNumber, filepath);
    fprintf('[run_logger]   label   : %s\n', label);
    if isfield(meta, 'source') && ~isempty(meta.source)
        fprintf('[run_logger]   source  : %s\n', meta.source);
    end
    if isfield(meta, 'comment') && ~isempty(meta.comment)
        fprintf('[run_logger]   comment : %s\n', meta.comment);
    end
end

% =====================================================================
%  HILFSFUNKTIONEN
% =====================================================================

function s = sanitizeLabel(s)
    s = char(string(s));
    s = regexprep(s, '[^a-zA-Z0-9_\-]', '_');
    s = lower(s);
    if numel(s) > 60
        s = s(1:60);
    end
    if isempty(s)
        s = 'unnamed';
    end
end

function n = nextRunNumber(runDir, prefix)
    pattern = sprintf('%s_*.mat', prefix);
    files   = dir(fullfile(runDir, pattern));
    maxN    = 0;
    re      = sprintf('^%s_(\\d+)_', regexptranslate('escape', prefix));
    for i = 1:numel(files)
        tok = regexp(files(i).name, re, 'tokens', 'once');
        if ~isempty(tok)
            num = str2double(tok{1});
            if ~isnan(num) && num > maxN
                maxN = num;
            end
        end
    end
    n = maxN + 1;
end

function h = getHostname()
    h = 'unknown';
    try
        [status, out] = system('hostname');
        if status == 0
            h = strtrim(out);
        end
    catch
    end
    if isempty(h)
        h = 'unknown';
    end
end

function hash = tryGetGitHash()
    hash = '';
    try
        [status, out] = system('git rev-parse --short HEAD');
        if status == 0
            hash = strtrim(out);
        end
    catch
    end
end

function name = getCallerName()
    name = '';
    try
        st = dbstack;
        if numel(st) >= 3
            name = st(3).name;
        end
    catch
    end
end

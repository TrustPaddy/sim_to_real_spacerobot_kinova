function setup_project()
%SETUP_PROJECT  Richtet MATLAB fuer dieses Repository ein. Einmal pro Sitzung ausfuehren.
%   - fuegt alle Code-Ordner zum MATLAB-Pfad hinzu
%   - wechselt in den Repository-Ordner. Die Simulink-Modelle laden die
%     Kinova-Meshes ueber den relativen Pfad ros_kortex/..., deshalb muessen
%     Training und Auswertung von hier aus laufen.
%   - legt Simulink-Cache und generierten Code in work/ ab (per .gitignore
%     ausgeschlossen)

root = fileparts(mfilename("fullpath"));
addpath(root, ...
    fullfile(root, "utils"), ...
    fullfile(root, "robot"), ...
    fullfile(root, "models"), ...
    fullfile(root, "training"), ...
    fullfile(root, "evaluation"), ...
    fullfile(root, "hardware", "deploy"), ...
    fullfile(root, "hardware", "playback"), ...
    fullfile(root, "hardware", "analysis"));
cd(root);

workDir = fullfile(root, "work");
if ~isfolder(workDir)
    mkdir(workDir);
end
Simulink.fileGenControl("set", "CacheFolder", workDir, "CodeGenFolder", workDir);

meshDir = fullfile(root, "ros_kortex", "kortex_description", "arms", "gen3", "7dof", "meshes");
if ~isfolder(meshDir)
    warning("setup_project:meshes", ...
        "Kinova-Meshes fehlen (%s). Die Simulink-Modelle brauchen sie, siehe README.", meshDir);
end

fprintf("SpaceKinova eingerichtet: %s\n", root);
end

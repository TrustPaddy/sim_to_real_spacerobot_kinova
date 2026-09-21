function p = sk_path(varargin)
%SK_PATH  Absoluter Pfad innerhalb dieses Repositories.
%   p = SK_PATH("robot", "SpaceKinova.urdf") liefert den vollen Pfad zu
%   robot/SpaceKinova.urdf, unabhaengig vom aktuellen Ordner.
%   Teilpfade mit "/" sind erlaubt, z. B. SK_PATH("SavedAgents/Torque/Circular/PPO.mat").

root = fileparts(fileparts(mfilename("fullpath")));
p = fullfile(root, varargin{:});
end

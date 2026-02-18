function outFile = make_spacekinova_urdf(kinovaUrdFile, outFile, varargin)
%MAKE_SPACEKINOVA_URDF  Create a SpaceKinova URDF (cube base + Kinova arm)
%
% outFile = make_spacekinova_urdf("kinova.urdf","SpaceKinova.urdf")
%
% This script:
%   1) Reads the Kinova URDF (any 7-DoF Kinova URDF should work)
%   2) Prefixes all Kinova link/joint names (avoids name collisions with base_link)
%   3) Adds a cube base_link (like in your report's SpaceRobot.urdf)
%   4) Connects base_link to the prefixed Kinova root link via a fixed joint
%
% Optional name-value pairs:
%   "Prefix"   : string, default "kinova_"
%   "MountXYZ" : 1x3 double, default [0 0 0.5]  (top of a 1m cube)
%   "MountRPY" : 1x3 double, default [0 0 0]
%   "BaseSize" : 1x3 double, default [1 1 1]    (cube dimensions)
%   "BaseMass" : scalar, default 65
%   "BaseInertiaDiag" : 1x3 double, default [8 8 8] (ixx,iyy,izz)
%
% Notes:
% - If your Kinova URDF already uses a prefix or unique base link name,
%   you can set Prefix="" to keep names unchanged (not recommended).
% - After generation:
%     robot = importrobot(outFile); robot.DataFormat="row"; showdetails(robot);

p = inputParser;
p.addRequired("kinovaUrdFile", @(s)ischar(s)||isstring(s));
p.addRequired("outFile", @(s)ischar(s)||isstring(s));
p.addParameter("Prefix","kinova_", @(s)ischar(s)||isstring(s));
p.addParameter("MountXYZ",[0 0 0.5], @(v)isnumeric(v)&&numel(v)==3);
p.addParameter("MountRPY",[0 0 0], @(v)isnumeric(v)&&numel(v)==3);
p.addParameter("BaseSize",[1 1 1], @(v)isnumeric(v)&&numel(v)==3);
p.addParameter("BaseMass",65, @(v)isnumeric(v)&&isscalar(v)&&v>0);
% Korrekt: I = (1/12)*M*(a^2+b^2) = (1/12)*65*2 = 10.833 fuer 1x1x1m, 65kg
p.addParameter("BaseInertiaDiag",[10.833 10.833 10.833], @(v)isnumeric(v)&&numel(v)==3);
p.parse(kinovaUrdFile, outFile, varargin{:});

kinovaUrdFile = string(p.Results.kinovaUrdFile);
outFile       = string(p.Results.outFile);
prefix        = string(p.Results.Prefix);
mountXYZ      = reshape(double(p.Results.MountXYZ),1,3);
mountRPY      = reshape(double(p.Results.MountRPY),1,3);
baseSize      = reshape(double(p.Results.BaseSize),1,3);
baseMass      = double(p.Results.BaseMass);
I             = reshape(double(p.Results.BaseInertiaDiag),1,3);

assert(isfile(kinovaUrdFile), "Kinova URDF not found: %s", kinovaUrdFile);

txt = fileread(kinovaUrdFile);

% --- Extract content inside <robot ...> ... </robot>
% Remove XML header if present
txt = regexprep(txt, '^\s*<\?xml[^>]*\?>\s*', '');
% Get robot name (optional)
robotName = regexp(txt, '<robot\s+name\s*=\s*"([^"]+)"', 'tokens', 'once');
if isempty(robotName)
    robotName = {"kinova"};
end

% Extract inner block between opening <robot ...> and closing </robot>
openTag = regexp(txt, '<robot[^>]*>', 'match', 'once');
if isempty(openTag)
    error("Input file does not contain a <robot ...> tag.");
end
inner = regexprep(txt, '.*?<robot[^>]*>', '', 'dotexceptnewline');
inner = regexprep(inner, '</robot>\s*$', '', 'dotexceptnewline');

% --- Prefix Kinova names to avoid collision with base_link
% We target only attributes that reference link/joint/transmission names (avoid materials/textures).
if strlength(prefix) > 0
    % link & joint declarations
    inner = regexprep(inner, '(<\s*link\s+name\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);
    inner = regexprep(inner, '(<\s*joint\s+name\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);

    % parent/child link refs
    inner = regexprep(inner, '(<\s*parent\s+link\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);
    inner = regexprep(inner, '(<\s*child\s+link\s*=\s*")([^"]+)(")',  ['$1' char(prefix) '$2$3']);

    % mimic joint refs
    inner = regexprep(inner, '(<\s*mimic\s+joint\s*=\s*")([^"]+)(")',  ['$1' char(prefix) '$2$3']);

    % transmissions: transmission name and joint name inside transmissions
    inner = regexprep(inner, '(<\s*transmission\s+name\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);
    inner = regexprep(inner, '(<\s*joint\s+name\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);

    % gazebo reference tags (optional, harmless if not present)
    inner = regexprep(inner, '(<\s*gazebo\s+reference\s*=\s*")([^"]+)(")', ['$1' char(prefix) '$2$3']);
end

% --- Determine Kinova root link (prefixed)
linkNames = regexp(inner, '<\s*link\s+name\s*=\s*"([^"]+)"', 'tokens');
linkNames = unique(string([linkNames{:}]));
childLinks = regexp(inner, '<\s*child\s+link\s*=\s*"([^"]+)"', 'tokens');
childLinks = unique(string([childLinks{:}]));
rootCandidates = setdiff(linkNames, childLinks);

if isempty(rootCandidates)
    warning("Could not determine root link from joints; defaulting to first link.");
    kinovaRoot = linkNames(1);
else
    kinovaRoot = rootCandidates(1);
end

% --- Build SpaceKinova URDF text
baseLink = sprintf([ ...
'  <!-- central cube base (from your SpaceRobot.urdf structure) -->\n' ...
'  <link name="base_link">\n' ...
'    <inertial>\n' ...
'      <origin xyz="0 0 0"/>\n' ...
'      <mass value="%.6g"/>\n' ...
'      <inertia ixx="%.6g" iyy="%.6g" izz="%.6g" ixy="0" ixz="0" iyz="0"/>\n' ...
'    </inertial>\n' ...
'    <visual>\n' ...
'      <origin xyz="0 0 0"/>\n' ...
'      <geometry>\n' ...
'        <box size="%.6g %.6g %.6g"/>\n' ...
'      </geometry>\n' ...
'      <material name="blue">\n' ...
'        <color rgba="0.2 0.2 0.8 1"/>\n' ...
'      </material>\n' ...
'    </visual>\n' ...
'    <collision>\n' ...
'      <origin xyz="0 0 0"/>\n' ...
'      <geometry>\n' ...
'        <box size="%.6g %.6g %.6g"/>\n' ...
'      </geometry>\n' ...
'    </collision>\n' ...
'  </link>\n\n'], ...
baseMass, I(1), I(2), I(3), baseSize(1), baseSize(2), baseSize(3), baseSize(1), baseSize(2), baseSize(3));

fixedJoint = sprintf([ ...
'  <!-- mount Kinova arm to cube base -->\n' ...
'  <joint name="base_to_kinova" type="fixed">\n' ...
'    <parent link="base_link"/>\n' ...
'    <child link="%s"/>\n' ...
'    <origin xyz="%.6g %.6g %.6g" rpy="%.6g %.6g %.6g"/>\n' ...
'  </joint>\n\n'], ...
kinovaRoot, mountXYZ(1), mountXYZ(2), mountXYZ(3), mountRPY(1), mountRPY(2), mountRPY(3));

header = sprintf('<?xml version="1.0"?>\n<robot name="space_kinova">\n\n');
footer = sprintf('</robot>\n');

outTxt = string(header) + string(baseLink) + string(fixedJoint) ...
       + "  <!-- Kinova (prefixed) -->" + newline + string(inner) + newline ...
       + string(footer);

% --- Write output
fid = fopen(outFile, 'w');
assert(fid>0, "Could not open output file: %s", outFile);
fwrite(fid, outTxt);
fclose(fid);

fprintf("Wrote SpaceKinova URDF: %s\n", outFile);
fprintf("Kinova root link (after prefix): %s\n", kinovaRoot);

% --- Validierung der generierten URDF ---
try
    testRobot = importrobot(char(outFile));
    testRobot.DataFormat = 'row';
    nBodies = numel(testRobot.Bodies);
    nDOF = numel(homeConfiguration(testRobot));
    fprintf("Validierung OK: %d Bodies, %d DOF\n", nBodies, nDOF);
    if nDOF < 7
        warning("Weniger als 7 DOF erkannt (%d). Kinova Gen3 URDF pruefen.", nDOF);
    end
catch ME
    warning("URDF-Validierung fehlgeschlagen: %s\nBitte manuell pruefen.", ME.message);
end

end

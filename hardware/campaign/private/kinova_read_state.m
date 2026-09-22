function st = kinova_read_state(apiHandle, cfg)
%KINOVA_READ_STATE  Liest Gelenk- und Kortex-Zustand in einem RefreshFeedback-Aufruf.
%   st.q_rad        7x1, wie V2.3 mit wrapPi auf (-pi, pi]
%   st.dq_rad       7x1
%   st.pose_real    1x6 [x y z theta_x theta_y theta_z] aus baseFb.tool_pose
%                   (Position in m, reales Base-Frame; NaN, falls nicht vorhanden)
%   st.twist_real   1x6 [lin_x lin_y lin_z ang_x ang_y ang_z] aus baseFb.tool_twist
%   st.fault        logisch, true bei gesetzten Fault-Bits
%   st.errCode      Rueckgabecode von RefreshFeedback
[errCode, baseFb, actuatorFb, ~] = kortexApiMexInterface('RefreshFeedback', apiHandle);
st.errCode = errCode;
if errCode ~= 0
    error('kinova_read_state:refresh', 'RefreshFeedback fehlgeschlagen (errorCode=%d).', errCode);
end
q_deg  = extract_actuator_field(actuatorFb, 'position', cfg.nJ);
dq_deg = extract_actuator_field(actuatorFb, 'velocity', cfg.nJ);
st.q_rad  = mod(deg2rad(q_deg(:)) + pi, 2*pi) - pi;
st.dq_rad = deg2rad(dq_deg(:));
st.pose_real  = nan(1, 6);
st.twist_real = nan(1, 6);
if ~isempty(baseFb) && isfield(baseFb, 'tool_pose')
    st.pose_real = extract_vec(baseFb.tool_pose, {'x','y','z','theta_x','theta_y','theta_z'});
end
if ~isempty(baseFb) && isfield(baseFb, 'tool_twist')
    st.twist_real = extract_vec(baseFb.tool_twist, ...
        {'linear_x','linear_y','linear_z','angular_x','angular_y','angular_z'});
end
st.fault = check_faults(actuatorFb, baseFb);
end

function v = extract_vec(s, names)
% Robust gegen Struct- oder Array-Form (wie deploy_agent_kinova_point.m).
v = nan(1, numel(names));
if isnumeric(s)
    n = min(numel(names), numel(s));
    v(1:n) = double(s(1:n));
elseif isstruct(s)
    for i = 1:numel(names)
        if isfield(s, names{i}), v(i) = double(s.(names{i})); end
    end
end
end

function vals = extract_actuator_field(actuatorFb, fieldName, nJ)
vals = zeros(1, nJ);
if isscalar(actuatorFb) && isfield(actuatorFb, fieldName) && numel(actuatorFb.(fieldName)) >= nJ
    vals = double(actuatorFb.(fieldName)(1:nJ));
    return;
end
for i = 1:min(nJ, numel(actuatorFb))
    if isfield(actuatorFb, fieldName)
        vals(i) = double(actuatorFb(i).(fieldName));
    end
end
end

function flag = check_faults(actuatorFb, baseFb)
flag = false;
for name = {'fault_bank_a', 'fault_bank_b'}
    if ~isempty(actuatorFb) && isfield(actuatorFb, name{1})
        for i = 1:numel(actuatorFb)
            if any(double(actuatorFb(i).(name{1})) ~= 0), flag = true; return; end
        end
    end
    if ~isempty(baseFb) && isfield(baseFb, name{1}) && any(double(baseFb.(name{1})) ~= 0)
        flag = true; return;
    end
end
end

function m = campaign_env_meta()
%CAMPAIGN_ENV_META  Umgebungsdaten fuer jeden Kampagnen-Log (Rechner, MATLAB, Git-Stand).
m = struct();
m.datetimeStart = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
m.matlabVersion = version;
m.hostname = strtrim(getenv('COMPUTERNAME'));
if isempty(m.hostname)
    [~, h] = system('hostname'); m.hostname = strtrim(h);
end
m.cpu = strtrim(getenv('PROCESSOR_IDENTIFIER'));
try
    [st, out] = system('powershell -NoProfile -Command "(Get-CimInstance Win32_Processor).Name"');
    if st == 0 && ~isempty(strtrim(out)), m.cpu = strtrim(out); end
catch
end
m.os = char(computer);
root = sk_path();
[st, h] = system(sprintf('git -C "%s" rev-parse --short HEAD', root));
if st == 0, m.gitHash = strtrim(h); else, m.gitHash = 'unknown'; end
[st, d] = system(sprintf('git -C "%s" status --porcelain --untracked-files=no', root));
m.gitDirty = st == 0 && ~isempty(strtrim(d));
end

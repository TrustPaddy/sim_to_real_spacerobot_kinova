function h = file_md5(path)
%FILE_MD5  MD5-Pruefsumme einer Datei als Hex-String (belegt, welche Agent-Datei lief).
fid = fopen(path, 'r');
assert(fid > 0, 'file_md5: Datei nicht lesbar: %s', path);
c = onCleanup(@() fclose(fid));
bytes = fread(fid, inf, '*uint8');
md = java.security.MessageDigest.getInstance('MD5');
md.update(bytes);
h = lower(reshape(dec2hex(typecast(md.digest, 'uint8'), 2).', 1, []));
end

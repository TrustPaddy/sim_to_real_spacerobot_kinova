function append_index_csv(csvPath, row)
%APPEND_INDEX_CSV  Haengt eine Zeile an die Kampagnen-Uebersicht an (legt Kopfzeile bei Bedarf an).
%   row: struct mit skalaren Feldern (Zahl, logisch oder char). Die Feldreihenfolge
%   bestimmt die Spalten. Die CSV dient nur der schnellen Uebersicht im Labor,
%   die Auswertung liest immer die .mat-Logs.
fn = fieldnames(row);
isNew = ~isfile(csvPath);
fid = fopen(csvPath, 'a');
assert(fid > 0, 'append_index_csv: %s nicht schreibbar', csvPath);
c = onCleanup(@() fclose(fid));
if isNew
    fprintf(fid, '%s\n', strjoin(fn, ','));
end
vals = cell(1, numel(fn));
for i = 1:numel(fn)
    v = row.(fn{i});
    if ischar(v) || isstring(v)
        vals{i} = ['"' strrep(char(v), '"', '''') '"'];
    elseif islogical(v)
        vals{i} = sprintf('%d', v);
    else
        vals{i} = sprintf('%.6g', v);
    end
end
fprintf(fid, '%s\n', strjoin(vals, ','));
end

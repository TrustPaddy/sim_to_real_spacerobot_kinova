function s = to_char_struct(s)
%TO_CHAR_STRUCT  Wandelt string-Werte rekursiv in char um.
%   Die Kampagnen-Logs sollen auch ausserhalb von MATLAB lesbar sein
%   (scipy.io.loadmat kann MATLAB-string-Objekte nicht lesen).
if isstruct(s)
    for i = 1:numel(s)
        fn = fieldnames(s);
        for j = 1:numel(fn)
            s(i).(fn{j}) = to_char_struct(s(i).(fn{j}));
        end
    end
elseif isstring(s)
    if isscalar(s)
        s = char(s);
    else
        s = cellstr(s);
    end
elseif iscell(s)
    for i = 1:numel(s)
        s{i} = to_char_struct(s{i});
    end
end
end

function exportMatStructsRecursive_safe(matFilePath, outputRoot)
    if nargin < 2
        outputRoot = '.';
    end

    vars = load(matFilePath);
    varNames = fieldnames(vars);

    for i = 1:numel(varNames)
        varName = varNames{i};
        varValue = vars.(varName);

        if isstruct(varValue)
            usedNames = containers.Map('KeyType', 'char', 'ValueType', 'any');
            recurseSafeExport(varValue, fullfile(outputRoot), varName, usedNames);
        else
            fprintf('Skipping non-struct variable: %s\n', varName);
        end
    end
end

function recurseSafeExport(s, parentPath, fieldName, usedNames)
    safeName = getSafeFolderName(fieldName, usedNames);
    currentPath = fullfile(parentPath, safeName);

    if ~exist(currentPath, 'dir')
        mkdir(currentPath);
    end

    % Save the true name to metadata for safe import
    fid = fopen(fullfile(currentPath, '_true_name.txt'), 'w');
    fprintf(fid, '%s', fieldName);
    fclose(fid);

    fields = fieldnames(s);
    isLeaf = all(~structfun(@isstruct, s));

    if isLeaf
        outputCell = [{'Name', 'Value'}];
        for i = 1:numel(fields)
            val = s.(fields{i});
            if isnumeric(val) || islogical(val)
                outVal = val;
            elseif ischar(val) || isstring(val)
                outVal = char(val);
            else
                outVal = '[unexportable]';
            end
            outputCell(end + 1, :) = {fields{i}, outVal}; %#ok<AGROW>
        end
        writecell(outputCell, fullfile(currentPath, 'data.csv'));
    else
        childNames = containers.Map('KeyType', 'char', 'ValueType', 'any');
        for i = 1:numel(fields)
            subfield = fields{i};
            val = s.(subfield);
            if isstruct(val)
                recurseSafeExport(val, currentPath, subfield, childNames);
            else
                fprintf('Skipping non-struct field: %s/%s\n', currentPath, subfield);
            end
        end
    end
end

function safeName = getSafeFolderName(name, usedNames)
    key = lower(name);
    if isKey(usedNames, key)
        suffix = sprintf('_%d', usedNames(key) + 1);
        usedNames(key) = usedNames(key) + 1;
        safeName = [name, suffix];
    else
        usedNames(key) = 0;
        safeName = name;
    end
end

function s = importStructFromFolder(rootFolder)
    s = recurseImportSafe(rootFolder);
end


function outStruct = recurseImportSafe(currentPath)
    outStruct = struct();

    folderContents = dir(currentPath);
    folderContents = folderContents(~ismember({folderContents.name}, {'.', '..'}));

    for i = 1:numel(folderContents)
        item = folderContents(i);
        fullPath = fullfile(currentPath, item.name);

        if item.isdir
            % Detect true field name
            trueNameFile = fullfile(fullPath, '_true_name.txt');
            if isfile(trueNameFile)
                fid = fopen(trueNameFile, 'r');
                trueName = strtrim(fgetl(fid));
                fclose(fid);
            else
                trueName = item.name;
            end

            outStruct.(trueName) = recurseImportSafe(fullPath);

        elseif strcmpi(item.name, 'data.csv')

            try
                raw = readcell(fullPath, 'Delimiter', ',');
            catch
                warning('Could not read CSV: %s', fullPath);
                continue;
            end

            if size(raw,2) < 2 || ~strcmpi(raw{1,1}, 'Name')
                warning('Skipping malformed CSV: %s', fullPath);
                continue;
            end

            % Data rows
            data = raw(2:end,:);

            % Process each row
            for r = 1:size(data,1)

                key = data{r,1};
                if isempty(key), continue; end

                key = string(key);
                key = strtrim(key);

                % Parse the row into vector with truncation
                rowValues = data(r,2:end);
                vec = parseCSVRowTruncate(rowValues);

                % Assign
                outStruct.(key) = vec;
            end
        end
    end
end


%% ============================
%  Convert CSV row to vector with trailing empty/missing truncated
% ============================

function vec = parseCSVRowTruncate(row)
    N = numel(row);

    % First, find last non-empty/non-missing index
    lastValidIdx = 0;
    for i = N:-1:1
        x = row{i};
        if ~(isempty(x) || (ischar(x) && all(isspace(x))) || ismissing(x))
            lastValidIdx = i;
            break;
        end
    end

    if lastValidIdx == 0
        % entire row empty → return empty numeric array
        vec = [];
        return;
    end

    % Truncate row to lastValidIdx
    row = row(1:lastValidIdx);

    % Preallocate
    numericVals = nan(1,lastValidIdx);
    strVals = strings(1,lastValidIdx);
    isNumeric = true;

    for i = 1:lastValidIdx
        x = row{i};

        if isempty(x) || (ischar(x) && all(isspace(x))) || ismissing(x)
            numericVals(i) = NaN;
            strVals(i) = "";
            continue;
        end

        if ischar(x) || isstring(x)
            s = string(strtrim(x));
            if startsWith(s,"[") && endsWith(s,"]")
                s = extractBetween(s,2,strlength(s)-1);
            end

            num = str2double(s);

            if ~isnan(num)
                numericVals(i) = num;
                strVals(i) = s;
            else
                isNumeric = false;
                strVals(i) = s;
            end
            continue;
        end

        if isnumeric(x)
            numericVals(i) = x;
            strVals(i) = string(x);
        else
            isNumeric = false;
            strVals(i) = string(x);
        end
    end

    if isNumeric
        vec = numericVals;
    else
        vec = strVals;
    end
end

function s = importStructFromFolder_safe(rootFolder)
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
            % Attempt to load true field name
            trueNameFile = fullfile(fullPath, '_true_name.txt');
            if isfile(trueNameFile)
                fid = fopen(trueNameFile, 'r');
                trueName = strtrim(fgetl(fid));
                fclose(fid);
            else
                trueName = item.name;
            end

            % Recurse
            outStruct.(trueName) = recurseImportSafe(fullPath);

        elseif strcmpi(item.name, 'data.csv')
            try
                data = readcell(fullPath);
            catch
                warning('Could not read CSV: %s', fullPath);
                continue;
            end

            if size(data, 2) < 2 || ~strcmpi(data{1,1}, 'Name')
                warning('Skipping malformed CSV: %s', fullPath);
                continue;
            end

            % Assign key-value pairs to this level of the struct
            for r = 2:size(data, 1)
                key = data{r, 1};
                val = data{r, 2};

                if ischar(val) || isstring(val)
                    numeric = str2double(val);
                    if ~isnan(numeric)
                        val = numeric;
                    else
                        val = char(val);
                    end
                end

                outStruct.(key) = val;
            end
        end
    end
end

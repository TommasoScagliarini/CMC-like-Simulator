function result = audit_ground_datum_raw(repoRoot, outputFile)
% Read original EPIC tables; summarize spatial datum without conversion/tools.
% Static IK vectors remain complete (including left subtalar/MTP coordinates).
if nargin < 1, repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))); end
if nargin < 2, outputFile = fullfile(tempdir, 'cmc_ground_datum_raw.json'); end
subjects = {'AB06', 'AB07'};
sessions = {'10_09_18', '10_14_18'};
staticTrials = {'static_01', 'static01'};
result = struct('scope', 'Original MAT read only; no OpenSim, converter or integration');
result.files = {};
for iSubject = 1:numel(subjects)
    for task = {'static', 'treadmill'}
        if strcmp(task{1}, 'static'), trial = staticTrials{iSubject}; else, trial = 'treadmill_01_01'; end
        for kind = {'markers', 'ik', 'fp', 'conditions'}
            relative = fullfile('models', [subjects{iSubject} '-raw'], sessions{iSubject}, task{1}, kind{1}, [trial '.mat']);
            path = fullfile(repoRoot, relative);
            record = struct('subject', subjects{iSubject}, 'task', task{1}, 'kind', kind{1}, ...
                'path', strrep(relative, '\', '/'), 'exists', isfile(path));
            record.variables = {};
            if record.exists
                loaded = load(path);
                names = fieldnames(loaded);
                for i = 1:numel(names)
                    value = loaded.(names{i});
                    entry = struct('name', names{i}, 'class', class(value), 'size', size(value), ...
                        'summary', summarize(value, strcmp(task{1}, 'static') && strcmp(kind{1}, 'ik')));
                    record.variables{end+1} = entry;
                end
            end
            result.files{end+1} = record;
        end
    end
end
fid = fopen(outputFile, 'w');
if fid < 0, error('Cannot write requested temporary diagnostic JSON.'); end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(result, 'PrettyPrint', true));
fprintf('Read-only datum raw audit: %d files, output %s\n', numel(result.files), outputFile);
end

function record = summarize(value, keepStaticCoordinateVectors)
record = struct('class', class(value), 'size', size(value));
if istable(value)
    record.description = value.Properties.Description;
    record.variable_units = value.Properties.VariableUnits;
    record.variable_descriptions = value.Properties.VariableDescriptions;
    record.user_data = value.Properties.UserData;
    names = value.Properties.VariableNames;
    record.columns = {};
    for i = 1:numel(names)
        column = value.(names{i});
        entry = struct('name', names{i}, 'summary', summarize(column, keepStaticCoordinateVectors));
        record.columns{end+1} = entry;
    end
elseif isstruct(value) && isscalar(value)
    names = fieldnames(value);
    record.fields = {};
    for i = 1:numel(names)
        record.fields{end+1} = struct('name', names{i}, 'summary', summarize(value.(names{i}), keepStaticCoordinateVectors));
    end
elseif isnumeric(value) || islogical(value)
    data = double(value);
    if ~isempty(data) && ismatrix(data)
        record.finite_count = sum(isfinite(data), 1);
        record.min = min(data, [], 1, 'omitnan');
        record.max = max(data, [], 1, 'omitnan');
        record.mean = mean(data, 1, 'omitnan');
        record.median = median(data, 1, 'omitnan');
        record.std = std(data, 0, 1, 'omitnan');
        record.first = data(1, :);
        record.last = data(end, :);
        if keepStaticCoordinateVectors && isvector(data)
            record.values = data(:).';
        end
    end
elseif ischar(value) || isstring(value)
    record.text = string(value);
end
end

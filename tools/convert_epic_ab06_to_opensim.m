function summary = convert_epic_ab06_to_opensim(varargin)
%CONVERT_EPIC_AB06_TO_OPENSIM Convert EPIC AB06 .mat trials to OpenSim files.
%
% Default smoke-test conversion:
%   convert_epic_ab06_to_opensim()
%
% Convert one trial:
%   convert_epic_ab06_to_opensim('Task','treadmill','Trial','treadmill_01_01')
%
% Convert every trial that has marker data:
%   convert_epic_ab06_to_opensim('Batch',true,'Task','')
%
% The AB06 .mat files in this project are MATLAB table/MCOS files. They are
% intentionally converted from MATLAB instead of scipy.io.loadmat.

repoRoot = defaultRepoRoot();
opts = parseOptions(repoRoot, varargin{:});

opts.MarkerNames = ab06MarkerNames();
opts.TargetCoordinates = ab06SeaseaCoordinates();
opts.SourceCoordinateNames = sourceCoordinateNames(opts.TargetCoordinates, opts.TargetModel);

trials = resolveTrials(opts);
summary = repmat(emptyResult(), numel(trials), 1);

for iTrial = 1:numel(trials)
    try
        summary(iTrial) = convertOneTrial(trials(iTrial), opts);
    catch ME
        if opts.StopOnError
            rethrow(ME);
        end
        warning('convert_epic_ab06_to_opensim:trialFailed', ...
            'Failed %s/%s: %s', trials(iTrial).Task, trials(iTrial).Trial, ME.message);
        result = emptyResult();
        result.Task = trials(iTrial).Task;
        result.Trial = trials(iTrial).Trial;
        result.Status = "failed";
        result.Message = string(ME.message);
        summary(iTrial) = result;
    end
end

printSummary(summary);

if nargout == 0
    clear summary;
end
end

function opts = parseOptions(repoRoot, varargin)
p = inputParser();
p.FunctionName = 'convert_epic_ab06_to_opensim';

addParameter(p, 'SubjectDir', fullfile(repoRoot, 'models', 'AB06-raw', '10_09_18'), @isTextScalar);
addParameter(p, 'Task', 'treadmill', @(x) isTextScalar(x) || isEmptyText(x));
addParameter(p, 'Trial', 'treadmill_01_01', @(x) isTextScalar(x) || isEmptyText(x));
addParameter(p, 'Batch', false, @isLogicalScalar);
addParameter(p, 'OutputDir', fullfile(repoRoot, 'models', 'AB06_SEASEA-raw', 'data', 'converted'), @isTextScalar);
addParameter(p, 'TargetModel', 'AB06_SEASEA', @isTextScalar);
addParameter(p, 'ModelFile', defaultModelFile(repoRoot), @isTextScalar);
addParameter(p, 'IkTemplate', fullfile(repoRoot, 'models', 'AB06_SEASEA-raw', 'osimxml', 'iksetup.xml'), @isTextScalar);
addParameter(p, 'IdTemplate', fullfile(repoRoot, 'models', 'AB06_SEASEA-raw', 'osimxml', 'idsetup.xml'), @isTextScalar);
addParameter(p, 'UseDatasetIKForID', false, @isLogicalScalar);
addParameter(p, 'MarkerUnits', 'auto', @(x) isTextScalar(x));
addParameter(p, 'MarkerScale', 1.0, @isNumericScalar);
addParameter(p, 'PointScale', 'auto', @(x) isNumericScalar(x) || isTextScalar(x));
addParameter(p, 'ForceScale', 1.0, @isNumericScalar);
addParameter(p, 'TorqueScale', 'auto', @(x) isNumericScalar(x) || isTextScalar(x));
addParameter(p, 'TorqueMode', 'free_vertical', @isTextScalar);
addParameter(p, 'IkAnglesInDegrees', true, @isLogicalScalar);
addParameter(p, 'DefaultMarkerRate', 100.0, @isNumericScalar);
addParameter(p, 'DefaultForceRate', 1000.0, @isNumericScalar);
addParameter(p, 'ForcePlateBodies', {'foot_l', 'calcn_r'}, @isCellstrLike);
addParameter(p, 'ForcePlatePrefixes', {'Treadmill_L', 'Treadmill_R'}, @isCellstrLike);
addParameter(p, 'AllowMissingMarkers', false, @isLogicalScalar);
addParameter(p, 'AllowMissingForcePlates', false, @isLogicalScalar);
addParameter(p, 'TrimToConditionRange', false, @isLogicalScalar);
addParameter(p, 'StopOnError', true, @isLogicalScalar);

parse(p, varargin{:});
opts = p.Results;

opts.SubjectDir = char(opts.SubjectDir);
opts.Task = char(opts.Task);
opts.Trial = char(opts.Trial);
opts.OutputDir = char(opts.OutputDir);
opts.TargetModel = char(opts.TargetModel);
opts.ModelFile = char(opts.ModelFile);
opts.IkTemplate = char(opts.IkTemplate);
opts.IdTemplate = char(opts.IdTemplate);
opts.MarkerUnits = char(opts.MarkerUnits);
opts.TorqueMode = char(opts.TorqueMode);
end

function path = defaultModelFile(repoRoot)
bundleModel = fullfile(repoRoot, 'models', 'AB06_SEASEA', 'Adjusted_newmarkers_pipeline_ready.osim');
rawCalibrated = fullfile(repoRoot, 'models', 'AB06_SEASEA-raw', 'osimxml', 'AB06_SEASEA_marker_calibrated.osim');
rawUncalibrated = fullfile(repoRoot, 'models', 'AB06_SEASEA-raw', 'osimxml', 'AB06_SEASEA.osim');
if isfile(bundleModel)
    path = bundleModel;
elseif isfile(rawCalibrated)
    path = rawCalibrated;
else
    path = rawUncalibrated;
end
end

function trials = resolveTrials(opts)
if opts.Batch
    if isempty(opts.Task)
        files = dir(fullfile(opts.SubjectDir, '*', 'markers', '*.mat'));
    else
        files = dir(fullfile(opts.SubjectDir, opts.Task, 'markers', '*.mat'));
    end
    if isempty(files)
        error('No marker .mat files found under %s.', opts.SubjectDir);
    end
    trials = repmat(struct('Task', '', 'Trial', ''), numel(files), 1);
    for iFile = 1:numel(files)
        markerDir = files(iFile).folder;
        taskDir = fileparts(markerDir);
        [~, taskName] = fileparts(taskDir);
        [~, trialName] = fileparts(files(iFile).name);
        trials(iFile).Task = taskName;
        trials(iFile).Trial = trialName;
    end
    return;
end

if isempty(opts.Trial)
    error('Trial must be provided when Batch is false.');
end

if isempty(opts.Task)
    files = dir(fullfile(opts.SubjectDir, '*', 'markers', [opts.Trial '.mat']));
    if isempty(files)
        error('Could not find marker file for trial %s.', opts.Trial);
    end
    if numel(files) > 1
        names = strings(numel(files), 1);
        for iFile = 1:numel(files)
            taskDir = fileparts(files(iFile).folder);
            [~, taskCandidate] = fileparts(taskDir);
            names(iFile) = string(taskCandidate);
        end
        error('Trial %s exists in multiple tasks: %s. Specify Task.', opts.Trial, strjoin(cellstr(names), ', '));
    end
    taskDir = fileparts(files(1).folder);
    [~, taskName] = fileparts(taskDir);
else
    taskName = opts.Task;
end

trials = struct('Task', taskName, 'Trial', opts.Trial);
end

function result = convertOneTrial(trial, opts)
result = emptyResult();
result.Task = string(trial.Task);
result.Trial = string(trial.Trial);

trialRoot = fullfile(opts.SubjectDir, trial.Task);
markerPath = fullfile(trialRoot, 'markers', [trial.Trial '.mat']);
fpPath = fullfile(trialRoot, 'fp', [trial.Trial '.mat']);
ikPath = fullfile(trialRoot, 'ik', [trial.Trial '.mat']);
conditionsPath = fullfile(trialRoot, 'conditions', [trial.Trial '.mat']);

if ~isfile(markerPath)
    error('Missing marker file: %s', markerPath);
end

outDir = fullfile(opts.OutputDir, trial.Task, trial.Trial);
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
result.OutputDir = string(outDir);

cond = loadConditions(conditionsPath);

[markerPayload, markerRaw] = loadPayload(markerPath, {'markers', 'markerData', 'markerTable', 'data'});
[markerTime, markerData] = extractMarkerData(markerPayload, markerRaw, opts, cond);
if opts.TrimToConditionRange
    [markerTime, markerData] = trimTimeRows(markerTime, markerData, cond);
end

trcPath = fullfile(outDir, [trial.Trial '.trc']);
writeTrc(trcPath, markerTime, markerData, opts.MarkerNames, opts);
result.TrcFile = string(trcPath);

ikMotPath = '';
if isfile(ikPath)
    [ikPayload, ikRaw] = loadPayload(ikPath, {'ik', 'inverseKinematics', 'coordinates', 'data'});
    [ikTime, ikData] = extractCoordinateData(ikPayload, ikRaw, opts, cond);
    if opts.TrimToConditionRange
        [ikTime, ikData] = trimTimeRows(ikTime, ikData, cond);
    end
    ikMotPath = fullfile(outDir, [trial.Trial '_ik_dataset_ab06_seasea.mot']);
    writeCoordinateMot(ikMotPath, ikTime, ikData, opts.TargetCoordinates);
    result.DatasetIkMotFile = string(ikMotPath);
else
    warning('convert_epic_ab06_to_opensim:missingIK', 'Missing IK file for %s/%s.', trial.Task, trial.Trial);
end

grfPath = '';
externalLoadsPath = '';
if isfile(fpPath)
    [fpPayload, fpRaw] = loadPayload(fpPath, {'fp', 'forceplate', 'forcePlates', 'grf', 'data'});
    [forceTime, grfData] = extractGrfData(fpPayload, fpRaw, opts, cond);
    if opts.TrimToConditionRange
        [forceTime, grfData] = trimTimeRows(forceTime, grfData, cond);
    end
    grfPath = fullfile(outDir, [trial.Trial '_grf.mot']);
    writeGrfMot(grfPath, forceTime, grfData);
    result.GrfMotFile = string(grfPath);

    externalLoadsPath = fullfile(outDir, [trial.Trial '_ExternalLoads.xml']);
    writeExternalLoadsXml(externalLoadsPath, grfPath, opts.ForcePlateBodies);
    result.ExternalLoadsFile = string(externalLoadsPath);
else
    warning('convert_epic_ab06_to_opensim:missingFP', ...
        'Missing force plate file for %s/%s. GRF MOT and ExternalLoads XML were skipped.', trial.Task, trial.Trial);
end

timeRange = chooseTimeRange(cond, markerTime);
ikSetupPath = fullfile(outDir, [trial.Trial '_iksetup.xml']);
writeIkSetup(ikSetupPath, opts, trcPath, outDir, [trial.Trial '_ik.mot'], timeRange);
result.IkSetupFile = string(ikSetupPath);

idSetupPath = '';
if ~isempty(externalLoadsPath)
    idSetupPath = fullfile(outDir, [trial.Trial '_idsetup.xml']);
    if opts.UseDatasetIKForID && ~isempty(ikMotPath)
        coordinatesFile = ikMotPath;
    else
        coordinatesFile = fullfile(outDir, [trial.Trial '_ik.mot']);
    end
    writeIdSetup(idSetupPath, opts, coordinatesFile, externalLoadsPath, outDir, [trial.Trial '_id.sto'], timeRange);
    result.IdSetupFile = string(idSetupPath);
end

result.Status = "ok";
result.Message = "converted";
fprintf('Converted %s/%s -> %s\n', trial.Task, trial.Trial, outDir);
end

function cond = loadConditions(path)
cond = struct('Start', NaN, 'End', NaN);
if ~isfile(path)
    return;
end

S = load(path);
if isfield(S, 'trialStarts')
    cond.Start = firstNumeric(S.trialStarts);
end
if isfield(S, 'trialEnds')
    cond.End = firstNumeric(S.trialEnds);
end
end

function [payload, raw] = loadPayload(path, preferredNames)
raw = load(path);
names = fieldnames(raw);
if isempty(names)
    error('No variables found in %s.', path);
end

for iName = 1:numel(preferredNames)
    idx = findName(names, preferredNames{iName});
    if idx > 0
        payload = raw.(names{idx});
        return;
    end
end

rank = zeros(numel(names), 1);
for iName = 1:numel(names)
    value = raw.(names{iName});
    rank(iName) = payloadRank(value);
end
[bestRank, bestIdx] = max(rank);
if bestRank <= 0
    error('Could not identify a table, struct, object, or numeric payload in %s.', path);
end
payload = raw.(names{bestIdx});
end

function rank = payloadRank(value)
rank = 0;
if istableLike(value)
    rank = 100000 + tableHeight(value);
elseif isstruct(value)
    rank = 50000 + numel(fieldnames(value));
elseif isnumeric(value) && ~isempty(value)
    rank = 10000 + numel(value);
elseif isobject(value)
    rank = 1000;
end
end

function [time, markerData] = extractMarkerData(payload, raw, opts, cond)
nRows = inferRows(payload, raw);
time = extractTimeVector(payload, raw, nRows, cond, opts.DefaultMarkerRate);

markerData = NaN(nRows, numel(opts.MarkerNames), 3);
missing = strings(0, 1);
for iMarker = 1:numel(opts.MarkerNames)
    markerName = opts.MarkerNames{iMarker};
    triplet = extractTriplet(payload, raw, markerName, nRows);
    if isempty(triplet)
        missing(end + 1, 1) = string(markerName); %#ok<AGROW>
        continue;
    end
    markerData(:, iMarker, :) = triplet .* opts.MarkerScale;
end

if ~isempty(missing)
    msg = sprintf('Missing marker columns: %s', strjoin(cellstr(missing), ', '));
    if opts.AllowMissingMarkers
        warning('convert_epic_ab06_to_opensim:missingMarkers', msg);
    else
        error(msg);
    end
end
end

function [time, coordData] = extractCoordinateData(payload, raw, opts, cond)
nRows = inferRows(payload, raw);
time = extractTimeVector(payload, raw, nRows, cond, opts.DefaultMarkerRate);

coordData = NaN(nRows, numel(opts.TargetCoordinates));
missing = strings(0, 1);
for iCoord = 1:numel(opts.TargetCoordinates)
    sourceName = opts.SourceCoordinateNames{iCoord};
    values = extractNamedVector(payload, raw, coordinateCandidates(sourceName), nRows);
    if isempty(values)
        missing(end + 1, 1) = string(sourceName); %#ok<AGROW>
        continue;
    end
    values = values(:);
    if ~opts.IkAnglesInDegrees && ~isTranslationalCoordinate(opts.TargetCoordinates{iCoord})
        values = rad2deg(values);
    end
    coordData(:, iCoord) = values;
end

if ~isempty(missing)
    error('Missing IK coordinate columns: %s', strjoin(cellstr(missing), ', '));
end
end

function [time, grfData] = extractGrfData(payload, raw, opts, cond)
nRows = inferRows(payload, raw);
time = extractTimeVector(payload, raw, nRows, cond, opts.DefaultForceRate);

force = cell(2, 1);
point = cell(2, 1);
torque = cell(2, 1);
missingPlates = [];
for iPlate = 1:2
    force{iPlate} = extractForcePlateTriplet(payload, raw, iPlate, 'force', nRows, opts);
    point{iPlate} = extractForcePlateTriplet(payload, raw, iPlate, 'point', nRows, opts);
    torque{iPlate} = extractForcePlateTriplet(payload, raw, iPlate, 'torque', nRows, opts);
    if isempty(force{iPlate}) || isempty(point{iPlate}) || isempty(torque{iPlate})
        missingPlates(end + 1) = iPlate; %#ok<AGROW>
    end
end

if ~isempty(missingPlates)
    msg = sprintf('Could not identify complete force/point/torque data for force plate(s): %s', num2str(missingPlates));
    if ~opts.AllowMissingForcePlates
        error(msg);
    end
    warning('convert_epic_ab06_to_opensim:missingForcePlate', msg);
    for iPlate = missingPlates
        if isempty(force{iPlate}), force{iPlate} = zeros(nRows, 3); end
        if isempty(point{iPlate}), point{iPlate} = zeros(nRows, 3); end
        if isempty(torque{iPlate}), torque{iPlate} = zeros(nRows, 3); end
    end
end

pointScale = resolveAutoScale(opts.PointScale, cat(1, point{:}), 20, 0.001);
torqueScale = resolveAutoScale(opts.TorqueScale, cat(1, torque{:}), 1000, 0.001);

force{1} = force{1} .* opts.ForceScale;
force{2} = force{2} .* opts.ForceScale;
point{1} = point{1} .* pointScale;
point{2} = point{2} .* pointScale;
torque{1} = torque{1} .* torqueScale;
torque{2} = torque{2} .* torqueScale;

[torque{1}, torque{2}] = convertTorqueForOpenSim(force, point, torque, opts.TorqueMode);

grfData = [force{1}, point{1}, force{2}, point{2}, torque{1}, torque{2}];
end

function [torque1, torque2] = convertTorqueForOpenSim(force, point, torque, mode)
% ExternalForce applies force at COP, so OpenSim already contributes r x F.
% EPIC treadmill moment channels behave like moments about the lab origin.
% Convert them to the free torque expected at the COP.
mode = lower(strtrim(char(mode)));
torqueOut = cell(2, 1);
for iPlate = 1:2
    switch mode
        case {'raw', 'asis', 'as_is'}
            torqueOut{iPlate} = torque{iPlate};
        case {'origin_to_free', 'free'}
            torqueOut{iPlate} = torque{iPlate} - cross(point{iPlate}, force{iPlate}, 2);
        case {'free_vertical', 'vertical_free'}
            freeTorque = torque{iPlate} - cross(point{iPlate}, force{iPlate}, 2);
            torqueOut{iPlate} = zeros(size(freeTorque));
            torqueOut{iPlate}(:, 2) = freeTorque(:, 2);
        case 'zero'
            torqueOut{iPlate} = zeros(size(torque{iPlate}));
        otherwise
            error('Unknown TorqueMode "%s". Use raw, origin_to_free, free_vertical, or zero.', mode);
    end
end
torque1 = torqueOut{1};
torque2 = torqueOut{2};
end

function triplet = extractForcePlateTriplet(payload, raw, plateIndex, kind, nRows, opts)
directLabels = opensimGrfTripletLabels(plateIndex, kind);
triplet = extractTripletByColumnNames(payload, raw, directLabels, nRows);
if ~isempty(triplet)
    return;
end

baseCandidates = forcePlateBaseCandidates(plateIndex, kind);
if isfield(opts, 'ForcePlatePrefixes') && numel(opts.ForcePlatePrefixes) >= plateIndex
    preferredPrefix = opts.ForcePlatePrefixes{plateIndex};
    if ~isempty(preferredPrefix)
        prefixCandidates = forcePlatePrefixCandidates(preferredPrefix, kind);
        baseCandidates = [prefixCandidates(:).', baseCandidates(:).'];
    end
end
triplet = extractTripletByBases(payload, raw, baseCandidates, nRows);
end

function labels = opensimGrfTripletLabels(plateIndex, kind)
switch kind
    case 'force'
        prefix = sprintf('ground_force%d_v', plateIndex);
    case 'point'
        prefix = sprintf('ground_force%d_p', plateIndex);
    case 'torque'
        prefix = sprintf('ground_torque%d_', plateIndex);
    otherwise
        error('Unknown GRF kind: %s', kind);
end
labels = {[prefix 'x'], [prefix 'y'], [prefix 'z']};
end

function bases = forcePlateBaseCandidates(plateIndex, kind)
p = plateIndex;
switch kind
    case 'force'
        words = {'force', 'forces', 'f', 'grf', 'v'};
        opensimBase = sprintf('ground_force%d_v', p);
    case 'point'
        words = {'point', 'points', 'cop', 'coposition', 'p', 'origin'};
        opensimBase = sprintf('ground_force%d_p', p);
    case 'torque'
        words = {'torque', 'torques', 'moment', 'moments', 'm', 't'};
        opensimBase = sprintf('ground_torque%d_', p);
    otherwise
        error('Unknown GRF kind: %s', kind);
end

bases = {opensimBase};
for iWord = 1:numel(words)
    w = words{iWord};
    bases{end + 1} = sprintf('fp%d_%s', p, w); %#ok<AGROW>
    bases{end + 1} = sprintf('fp%d%s', p, w); %#ok<AGROW>
    bases{end + 1} = sprintf('%s%d', w, p); %#ok<AGROW>
    bases{end + 1} = sprintf('%s_%d', w, p); %#ok<AGROW>
    bases{end + 1} = sprintf('forceplate%d_%s', p, w); %#ok<AGROW>
    bases{end + 1} = sprintf('plate%d_%s', p, w); %#ok<AGROW>
end
end

function bases = forcePlatePrefixCandidates(prefix, kind)
switch kind
    case 'force'
        suffixes = {'v', 'force', 'forces', 'f', 'grf'};
    case 'point'
        suffixes = {'p', 'point', 'points', 'cop'};
    case 'torque'
        suffixes = {'moment', 'moments', 'torque', 'torques', 'm', 't'};
    otherwise
        error('Unknown GRF kind: %s', kind);
end

bases = cell(numel(suffixes), 1);
for iSuffix = 1:numel(suffixes)
    bases{iSuffix} = sprintf('%s_%s', prefix, suffixes{iSuffix});
end
end

function triplet = extractTriplet(payload, raw, markerName, nRows)
triplet = extractTripletByBases(payload, raw, markerBaseCandidates(markerName), nRows);
end

function bases = markerBaseCandidates(markerName)
compact = strrep(markerName, '_', '');
bases = {markerName, lower(markerName), upper(markerName), compact, lower(compact), upper(compact)};
end

function triplet = extractTripletByBases(payload, raw, baseCandidates, nRows)
triplet = [];

triplet = extractTripletFromLabeledArray(payload, baseCandidates, nRows);
if isempty(triplet)
    triplet = extractTripletFromLabeledArray(raw, baseCandidates, nRows);
end
if ~isempty(triplet)
    return;
end

for iBase = 1:numel(baseCandidates)
    base = baseCandidates{iBase};
    matrix = extractNamedMatrix(payload, raw, {base}, nRows);
    if ~isempty(matrix) && size(matrix, 2) >= 3
        triplet = matrix(:, 1:3);
        return;
    end

    labels = componentLabels(base);
    byColumns = extractTripletByColumnNames(payload, raw, labels, nRows);
    if ~isempty(byColumns)
        triplet = byColumns;
        return;
    end
end
end

function triplet = extractTripletByColumnNames(payload, raw, labels, nRows)
triplet = [];
columns = NaN(nRows, 3);
for iComponent = 1:3
    values = extractNamedVector(payload, raw, labels{iComponent}, nRows);
    if isempty(values)
        return;
    end
    columns(:, iComponent) = values(:);
end
triplet = columns;
end

function labels = componentLabels(base)
labels = cell(3, 1);
components = {'x', 'y', 'z'};
indices = {'1', '2', '3'};
for iComponent = 1:3
    c = components{iComponent};
    idx = indices{iComponent};
    labels{iComponent} = { ...
        [base '_' c], [base '_' upper(c)], [base c], [base upper(c)], ...
        [base '.' c], [base '.' upper(c)], [base idx], [base '_' idx]};
end
end

function candidates = coordinateCandidates(name)
candidates = {name, lower(name), upper(name), strrep(name, '_', '')};
end

function values = extractNamedVector(payload, raw, candidates, nRows)
values = [];
if ischar(candidates) || isstring(candidates)
    candidates = cellstr(string(candidates));
end

member = extractNamedMember(payload, candidates);
if isempty(member)
    member = extractNamedMember(raw, candidates);
end

if ~isempty(member)
    vector = coerceVector(member, nRows);
    if ~isempty(vector)
        values = vector;
        return;
    end
end

values = extractVectorFromLabeledArray(payload, candidates, nRows);
if isempty(values)
    values = extractVectorFromLabeledArray(raw, candidates, nRows);
end
if ~isempty(values)
    return;
end

% Try one recursive pass through obvious container fields.
containers = namedMembers(payload);
for iMember = 1:numel(containers)
    member = containers(iMember).Value;
    if isContainerLike(member)
        values = extractNamedVector(member, struct(), candidates, nRows);
        if ~isempty(values)
            return;
        end
    end
end
end

function values = extractVectorFromLabeledArray(data, candidates, nRows)
values = [];
[labels, array] = labeledArray(data);
if isempty(labels) || isempty(array)
    return;
end

matrix = coerceMatrix(array, nRows);
if isempty(matrix) || size(matrix, 2) ~= numel(labels)
    return;
end

idx = findName(labels, candidates{1});
for iCandidate = 2:numel(candidates)
    if idx > 0
        break;
    end
    idx = findName(labels, candidates{iCandidate});
end
if idx > 0
    values = matrix(:, idx);
end
end

function triplet = extractTripletFromLabeledArray(data, candidates, nRows)
triplet = [];
[labels, array] = labeledArray(data);
if isempty(labels) || isempty(array)
    return;
end

idx = findName(labels, candidates{1});
for iCandidate = 2:numel(candidates)
    if idx > 0
        break;
    end
    idx = findName(labels, candidates{iCandidate});
end
if idx <= 0
    return;
end

if isnumeric(array) || islogical(array)
    if ndims(array) == 3 && size(array, 1) == nRows && size(array, 2) == 3 && size(array, 3) >= idx
        triplet = squeeze(double(array(:, :, idx)));
        return;
    elseif ndims(array) == 3 && size(array, 1) == nRows && size(array, 2) >= idx && size(array, 3) == 3
        triplet = squeeze(double(array(:, idx, :)));
        return;
    end
end

matrix = coerceMatrix(array, nRows);
if isempty(matrix)
    return;
end
if size(matrix, 2) >= numel(labels) * 3
    cols = (idx - 1) * 3 + (1:3);
    triplet = matrix(:, cols);
end
end

function [labels, array] = labeledArray(data)
labels = {};
array = [];
if isempty(data) || ~(isstruct(data) || isobject(data) || istableLike(data))
    return;
end

labelMember = extractNamedMember(data, {'labels', 'label', 'names', 'name', ...
    'columns', 'columnLabels', 'headers', 'colheaders'});
arrayMember = extractNamedMember(data, {'data', 'Data', 'values', 'Values', ...
    'matrix', 'array', 'Array'});
if isempty(labelMember) || isempty(arrayMember)
    return;
end

labels = coerceLabels(labelMember);
array = arrayMember;
end

function labels = coerceLabels(value)
labels = {};
if iscell(value)
    labels = cellfun(@char, value(:), 'UniformOutput', false);
elseif isstring(value)
    labels = cellstr(value(:));
elseif ischar(value)
    labels = cellstr(value);
elseif istableLike(value) && width(value) == 1
    labels = coerceLabels(value{:, 1});
end
end

function matrix = extractNamedMatrix(payload, raw, candidates, nRows)
matrix = [];
if ischar(candidates) || isstring(candidates)
    candidates = cellstr(string(candidates));
end

member = extractNamedMember(payload, candidates);
if isempty(member)
    member = extractNamedMember(raw, candidates);
end
if isempty(member)
    containers = namedMembers(payload);
    for iMember = 1:numel(containers)
        child = containers(iMember).Value;
        if isContainerLike(child)
            matrix = extractNamedMatrix(child, struct(), candidates, nRows);
            if ~isempty(matrix)
                return;
            end
        end
    end
    return;
end

matrix = coerceMatrix(member, nRows);
end

function value = extractNamedMember(data, candidates)
value = [];
members = namedMembers(data);
if isempty(members)
    return;
end

names = {members.Name};
for iCandidate = 1:numel(candidates)
    idx = findName(names, candidates{iCandidate});
    if idx > 0
        value = members(idx).Value;
        return;
    end
end
end

function members = namedMembers(data)
members = struct('Name', {}, 'Value', {});
if isempty(data)
    return;
end

if istableLike(data)
    names = data.Properties.VariableNames;
    members = repmat(struct('Name', '', 'Value', []), numel(names), 1);
    for iName = 1:numel(names)
        members(iName).Name = names{iName};
        members(iName).Value = data.(names{iName});
    end
elseif isstruct(data)
    names = fieldnames(data);
    members = repmat(struct('Name', '', 'Value', []), numel(names), 1);
    for iName = 1:numel(names)
        members(iName).Name = names{iName};
        members(iName).Value = data.(names{iName});
    end
elseif isa(data, 'timeseries')
    members = struct('Name', {'time', 'data'}, 'Value', {data.Time, data.Data});
elseif isobject(data)
    try
        names = properties(data);
    catch
        names = {};
    end
    members = repmat(struct('Name', '', 'Value', []), numel(names), 1);
    for iName = 1:numel(names)
        members(iName).Name = names{iName};
        try
            members(iName).Value = data.(names{iName});
        catch
            members(iName).Value = [];
        end
    end
end
end

function idx = findName(names, candidate)
idx = 0;
target = normalizeName(candidate);
for iName = 1:numel(names)
    if strcmp(normalizeName(names{iName}), target)
        idx = iName;
        return;
    end
end
end

function out = normalizeName(value)
out = lower(regexprep(char(string(value)), '[^A-Za-z0-9]', ''));
end

function time = extractTimeVector(payload, raw, nRows, cond, defaultRate)
timeCandidates = {'time', 'Time', 'times', 'seconds', 'sec', 't'};

if istimetableLike(payload)
    try
        rowTimes = payload.Properties.RowTimes;
        time = convertTimeLike(rowTimes);
        if numel(time) == nRows
            time = time(:);
            return;
        end
    catch
    end
end

time = extractNamedVector(payload, raw, timeCandidates, nRows);
if ~isempty(time)
    time = time(:);
    return;
end

if isfinite(cond.Start) && isfinite(cond.End) && nRows > 1
    time = linspace(cond.Start, cond.End, nRows).';
else
    time = ((0:nRows - 1).' ./ defaultRate);
end
end

function time = convertTimeLike(value)
if isduration(value)
    time = seconds(value);
elseif isdatetime(value)
    time = seconds(value - value(1));
else
    time = double(value);
end
time = time(:);
end

function nRows = inferRows(payload, raw)
nRows = inferRowsSingle(payload);
if nRows > 0
    return;
end
nRows = inferRowsSingle(raw);
if nRows > 0
    return;
end
error('Could not infer the number of samples in the MATLAB payload.');
end

function nRows = inferRowsSingle(data)
nRows = 0;
if isempty(data)
    return;
end
if istableLike(data)
    nRows = tableHeight(data);
elseif isnumeric(data) || islogical(data)
    if isvector(data)
        nRows = numel(data);
    else
        nRows = size(data, 1);
    end
elseif isa(data, 'timeseries')
    nRows = size(data.Data, 1);
elseif isstruct(data) || isobject(data)
    members = namedMembers(data);
    for iMember = 1:numel(members)
        childRows = inferRowsSingle(members(iMember).Value);
        if childRows > 1
            nRows = childRows;
            return;
        end
    end
end
end

function n = tableHeight(value)
try
    n = height(value);
catch
    n = size(value, 1);
end
end

function vector = coerceVector(value, nRows)
vector = [];
if isa(value, 'timeseries')
    value = value.Data;
end
if istableLike(value)
    if width(value) == 1
        value = value{:, 1};
    else
        return;
    end
end
if iscell(value)
    try
        value = cell2mat(value);
    catch
        return;
    end
end
if isnumeric(value) || islogical(value)
    if isvector(value) && numel(value) == nRows
        vector = double(value(:));
    elseif size(value, 1) == nRows && size(value, 2) == 1
        vector = double(value(:, 1));
    end
end
end

function matrix = coerceMatrix(value, nRows)
matrix = [];
if isa(value, 'timeseries')
    value = value.Data;
end
if istableLike(value)
    try
        value = table2array(value);
    catch
        return;
    end
end
if iscell(value)
    try
        value = cell2mat(value);
    catch
        return;
    end
end
if isnumeric(value) || islogical(value)
    if size(value, 1) == nRows && size(value, 2) >= 1
        matrix = double(value);
    elseif ndims(value) == 3 && size(value, 1) == nRows
        matrix = reshape(double(value), nRows, []);
    end
end
end

function [timeOut, dataOut] = trimTimeRows(time, data, cond)
timeOut = time;
dataOut = data;
if ~(isfinite(cond.Start) && isfinite(cond.End))
    return;
end
mask = time >= cond.Start & time <= cond.End;
if ~any(mask)
    warning('convert_epic_ab06_to_opensim:trimEmpty', ...
        'Condition time range %.6g %.6g does not overlap data time %.6g %.6g.', ...
        cond.Start, cond.End, time(1), time(end));
    return;
end
timeOut = time(mask);
if ndims(data) == 3
    dataOut = data(mask, :, :);
else
    dataOut = data(mask, :);
end
end

function timeRange = chooseTimeRange(cond, fallbackTime)
if isfinite(cond.Start) && isfinite(cond.End)
    timeRange = [cond.Start, cond.End];
else
    timeRange = [fallbackTime(1), fallbackTime(end)];
end
end

function writeTrc(path, time, markerData, markerNames, opts)
units = opts.MarkerUnits;
if strcmpi(units, 'auto')
    units = inferMarkerUnits(markerData);
end

dataRate = inferRate(time, opts.DefaultMarkerRate);
numFrames = numel(time);
numMarkers = numel(markerNames);

fid = fopen(path, 'w');
if fid < 0
    error('Could not write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'PathFileType\t4\t(X/Y/Z)\t%s\n', normalizePath(path));
fprintf(fid, 'DataRate\tCameraRate\tNumFrames\tNumMarkers\tUnits\tOrigDataRate\tOrigDataStartFrame\tOrigNumFrames\n');
fprintf(fid, '%.10g\t%.10g\t%d\t%d\t%s\t%.10g\t%d\t%d\n', ...
    dataRate, dataRate, numFrames, numMarkers, units, dataRate, 1, numFrames);

fprintf(fid, 'Frame#\tTime');
for iMarker = 1:numMarkers
    fprintf(fid, '\t%s\t\t', markerNames{iMarker});
end
fprintf(fid, '\n');

fprintf(fid, '\t');
for iMarker = 1:numMarkers
    fprintf(fid, '\tX%d\tY%d\tZ%d', iMarker, iMarker, iMarker);
end
fprintf(fid, '\n');

for iFrame = 1:numFrames
    fprintf(fid, '%d\t%.10f', iFrame, time(iFrame));
    for iMarker = 1:numMarkers
        xyz = squeeze(markerData(iFrame, iMarker, :));
        fprintf(fid, '\t%.10f\t%.10f\t%.10f', xyz(1), xyz(2), xyz(3));
    end
    fprintf(fid, '\n');
end
end

function writeCoordinateMot(path, time, coordData, coordNames)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Coordinates\n');
fprintf(fid, 'version=1\n');
fprintf(fid, 'nRows=%d\n', numel(time));
fprintf(fid, 'nColumns=%d\n', numel(coordNames) + 1);
fprintf(fid, 'inDegrees=yes\n\n');
fprintf(fid, 'Units are S.I. units (second, meters, Newtons, ...)\n');
fprintf(fid, 'Angles are in degrees.\n\n');
fprintf(fid, 'endheader\n');

fprintf(fid, 'time');
for iCoord = 1:numel(coordNames)
    fprintf(fid, '\t%s', coordNames{iCoord});
end
fprintf(fid, '\n');

for iRow = 1:numel(time)
    fprintf(fid, '%.10f', time(iRow));
    fprintf(fid, '\t%.10f', coordData(iRow, :));
    fprintf(fid, '\n');
end
end

function writeGrfMot(path, time, grfData)
labels = grfLabels();
fid = fopen(path, 'w');
if fid < 0
    error('Could not write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'name %s\n', normalizePath(path));
fprintf(fid, 'datacolumns %d\n', numel(labels) + 1);
fprintf(fid, 'datarows %d\n', numel(time));
fprintf(fid, 'range %.10g %.10g\n', time(1), time(end));
fprintf(fid, 'endheader\n\n');

fprintf(fid, 'time');
for iLabel = 1:numel(labels)
    fprintf(fid, '\t%s', labels{iLabel});
end
fprintf(fid, '\n');

for iRow = 1:numel(time)
    fprintf(fid, '%.10f', time(iRow));
    fprintf(fid, '\t%.10f', grfData(iRow, :));
    fprintf(fid, '\n');
end
end

function labels = grfLabels()
labels = { ...
    'ground_force1_vx', 'ground_force1_vy', 'ground_force1_vz', ...
    'ground_force1_px', 'ground_force1_py', 'ground_force1_pz', ...
    'ground_force2_vx', 'ground_force2_vy', 'ground_force2_vz', ...
    'ground_force2_px', 'ground_force2_py', 'ground_force2_pz', ...
    'ground_torque1_x', 'ground_torque1_y', 'ground_torque1_z', ...
    'ground_torque2_x', 'ground_torque2_y', 'ground_torque2_z'};
end

function writeExternalLoadsXml(path, grfPath, bodies)
if numel(bodies) < 2
    error('ForcePlateBodies must contain two body names.');
end

fid = fopen(path, 'w');
if fid < 0
    error('Could not write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '<?xml version="1.0" encoding="UTF-8"?>\n');
fprintf(fid, '<OpenSimDocument Version="40000">\n');
fprintf(fid, '  <ExternalLoads name="externalloads">\n');
fprintf(fid, '    <objects>\n');
writeExternalForce(fid, 'externalforce_L', bodies{1}, 1, grfPath);
writeExternalForce(fid, 'externalforce_R', bodies{2}, 2, grfPath);
fprintf(fid, '    </objects>\n');
fprintf(fid, '    <groups/>\n');
fprintf(fid, '    <datafile>%s</datafile>\n', xmlEscape(normalizePath(grfPath)));
fprintf(fid, '  </ExternalLoads>\n');
fprintf(fid, '</OpenSimDocument>\n');
end

function writeExternalForce(fid, name, body, plateIndex, grfPath)
fprintf(fid, '      <ExternalForce name="%s">\n', xmlEscape(name));
fprintf(fid, '        <applied_to_body>%s</applied_to_body>\n', xmlEscape(body));
fprintf(fid, '        <force_expressed_in_body>ground</force_expressed_in_body>\n');
fprintf(fid, '        <point_expressed_in_body>ground</point_expressed_in_body>\n');
fprintf(fid, '        <force_identifier>ground_force%d_v</force_identifier>\n', plateIndex);
fprintf(fid, '        <point_identifier>ground_force%d_p</point_identifier>\n', plateIndex);
fprintf(fid, '        <torque_identifier>ground_torque%d_</torque_identifier>\n', plateIndex);
fprintf(fid, '        <data_source_name>%s</data_source_name>\n', xmlEscape(normalizePath(grfPath)));
fprintf(fid, '      </ExternalForce>\n');
end

function writeIkSetup(path, opts, trcPath, resultsDir, outputMotionFile, timeRange)
if isfile(opts.IkTemplate)
    text = fileread(opts.IkTemplate);
else
    text = minimalIkSetupText();
end

text = setXmlTag(text, 'results_directory', normalizePath(resultsDir), 'InverseKinematicsTool');
text = setXmlTag(text, 'model_file', normalizePath(opts.ModelFile), 'InverseKinematicsTool');
text = setXmlTag(text, 'marker_file', normalizePath(trcPath), 'InverseKinematicsTool');
text = setXmlTag(text, 'coordinate_file', 'Unassigned', 'InverseKinematicsTool');
text = setXmlTag(text, 'time_range', sprintf('%.10g %.10g', timeRange(1), timeRange(2)), 'InverseKinematicsTool');
text = setXmlTag(text, 'output_motion_file', outputMotionFile, 'InverseKinematicsTool');
writeTextFile(path, text);
end

function writeIdSetup(path, opts, coordinatesFile, externalLoadsPath, resultsDir, outputForceFile, timeRange)
if isfile(opts.IdTemplate)
    text = fileread(opts.IdTemplate);
else
    text = minimalIdSetupText();
end

text = setXmlTag(text, 'results_directory', normalizePath(resultsDir), 'InverseDynamicsTool');
text = setXmlTag(text, 'model_file', normalizePath(opts.ModelFile), 'InverseDynamicsTool');
text = setXmlTag(text, 'time_range', sprintf('%.10g %.10g', timeRange(1), timeRange(2)), 'InverseDynamicsTool');
text = setXmlTag(text, 'external_loads_file', normalizePath(externalLoadsPath), 'InverseDynamicsTool');
text = setXmlTag(text, 'coordinates_file', normalizePath(coordinatesFile), 'InverseDynamicsTool');
text = setXmlTag(text, 'output_gen_force_file', outputForceFile, 'InverseDynamicsTool');
writeTextFile(path, text);
end

function text = setXmlTag(text, tag, value, parentTag)
value = xmlEscape(value);
pattern = ['<' tag '>(.*?)</' tag '>'];
[~, ~, tokenExtents] = regexp(text, pattern, 'start', 'end', 'tokenExtents', 'once');
if ~isempty(tokenExtents)
    if iscell(tokenExtents)
        tokenExtents = tokenExtents{1};
    end
    text = [text(1:tokenExtents(1) - 1), value, text(tokenExtents(2) + 1:end)];
    return;
end

closing = ['</' parentTag '>'];
idx = strfind(text, closing);
if isempty(idx)
    error('Could not find <%s> or closing </%s> in setup XML.', tag, parentTag);
end
insert = sprintf('    <%s>%s</%s>\n', tag, value, tag);
text = [text(1:idx(1) - 1), insert, text(idx(1):end)];
end

function text = minimalIkSetupText()
text = [ ...
    '<?xml version="1.0" encoding="utf-8"?>' newline ...
    '<OpenSimDocument Version="40000">' newline ...
    '  <InverseKinematicsTool>' newline ...
    '    <results_directory>.</results_directory>' newline ...
    '    <model_file>Unassigned</model_file>' newline ...
    '    <constraint_weight>Inf</constraint_weight>' newline ...
    '    <accuracy>1e-05</accuracy>' newline ...
    '    <IKTaskSet><objects/><groups/></IKTaskSet>' newline ...
    '    <marker_file>Unassigned</marker_file>' newline ...
    '    <coordinate_file>Unassigned</coordinate_file>' newline ...
    '    <time_range>Unassigned</time_range>' newline ...
    '    <report_errors>true</report_errors>' newline ...
    '    <output_motion_file>ik.mot</output_motion_file>' newline ...
    '  </InverseKinematicsTool>' newline ...
    '</OpenSimDocument>' newline];
end

function text = minimalIdSetupText()
text = [ ...
    '<?xml version="1.0" encoding="utf-8"?>' newline ...
    '<OpenSimDocument Version="40000">' newline ...
    '  <InverseDynamicsTool>' newline ...
    '    <results_directory>.</results_directory>' newline ...
    '    <model_file>Unassigned</model_file>' newline ...
    '    <time_range>Unassigned</time_range>' newline ...
    '    <forces_to_exclude>Muscles</forces_to_exclude>' newline ...
    '    <external_loads_file>Unassigned</external_loads_file>' newline ...
    '    <coordinates_file>Unassigned</coordinates_file>' newline ...
    '    <lowpass_cutoff_frequency_for_coordinates>6</lowpass_cutoff_frequency_for_coordinates>' newline ...
    '    <output_gen_force_file>id.sto</output_gen_force_file>' newline ...
    '  </InverseDynamicsTool>' newline ...
    '</OpenSimDocument>' newline];
end

function writeTextFile(path, text)
fid = fopen(path, 'w');
if fid < 0
    error('Could not write %s.', path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', text);
end

function units = inferMarkerUnits(markerData)
finiteValues = markerData(isfinite(markerData));
if isempty(finiteValues)
    units = 'm';
    return;
end
typicalMagnitude = median(abs(finiteValues));
if typicalMagnitude > 20
    units = 'mm';
else
    units = 'm';
end
end

function scale = resolveAutoScale(value, data, threshold, autoScale)
if isnumeric(value)
    scale = value;
    return;
end
if ~strcmpi(char(value), 'auto')
    error('Scale value must be numeric or ''auto''.');
end

finiteValues = data(isfinite(data));
if isempty(finiteValues)
    scale = 1.0;
    return;
end
typicalMagnitude = median(abs(finiteValues));
if typicalMagnitude > threshold
    scale = autoScale;
else
    scale = 1.0;
end
end

function rate = inferRate(time, fallbackRate)
if numel(time) < 2
    rate = fallbackRate;
    return;
end
dt = diff(time(:));
dt = dt(isfinite(dt) & dt > 0);
if isempty(dt)
    rate = fallbackRate;
else
    rate = 1.0 / median(dt);
end
end

function names = ab06MarkerNames()
names = { ...
    'R_ASIS', 'L_ASIS', 'L_PSIS', ...
    'R_Thigh_Upper', 'R_Thigh_Front', 'R_Thigh_Rear', 'R_Knee_Lat', ...
    'R_Shank_Upper', 'R_Shank_Front', 'R_Shank_Rear', 'R_Ankle_Lat', ...
    'R_Heel', 'R_Toe_Lat', 'R_Toe_Med', 'R_Toe_Tip', ...
    'L_Thigh_Upper', 'L_Thigh_Front', 'L_Thigh_Rear', 'L_Knee_Lat', ...
    'L_Shank_Upper', 'L_Shank_Front', 'L_Shank_Rear', 'L_Ankle_Lat', ...
    'L_Heel', 'L_Toe_Lat', 'L_Toe_Med', 'L_Toe_Tip', 'R_PSIS'};
end

function names = ab06SeaseaCoordinates()
names = { ...
    'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', ...
    'pelvis_tx', 'pelvis_ty', 'pelvis_tz', ...
    'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', ...
    'knee_angle_r', 'ankle_angle_r', 'subtalar_angle_r', 'mtp_angle_r', ...
    'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', ...
    'pros_knee_angle', 'pros_ankle_angle', ...
    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'};
end

function sourceNames = sourceCoordinateNames(targetNames, targetModel)
sourceNames = targetNames;
if contains(lower(char(targetModel)), 'seasea')
    sourceNames(strcmp(targetNames, 'pros_knee_angle')) = {'knee_angle_l'};
    sourceNames(strcmp(targetNames, 'pros_ankle_angle')) = {'ankle_angle_l'};
end
end

function tf = isTranslationalCoordinate(name)
tf = any(strcmp(name, {'pelvis_tx', 'pelvis_ty', 'pelvis_tz'}));
end

function result = emptyResult()
result = struct( ...
    'Task', "", ...
    'Trial', "", ...
    'Status', "", ...
    'Message', "", ...
    'OutputDir', "", ...
    'TrcFile', "", ...
    'GrfMotFile', "", ...
    'ExternalLoadsFile', "", ...
    'DatasetIkMotFile', "", ...
    'IkSetupFile', "", ...
    'IdSetupFile', "");
end

function printSummary(summary)
if isempty(summary)
    return;
end
statuses = string({summary.Status});
ok = sum(statuses == "ok");
failed = numel(summary) - ok;
fprintf('\nConversion summary: %d ok, %d failed.\n', ok, failed);
for i = 1:numel(summary)
    fprintf('  %-10s %-32s %s\n', char(summary(i).Task), char(summary(i).Trial), char(summary(i).Status));
end
end

function path = normalizePath(path)
path = char(path);
path = strrep(path, '\', '/');
end

function text = xmlEscape(value)
text = char(value);
text = strrep(text, '&', '&amp;');
text = strrep(text, '<', '&lt;');
text = strrep(text, '>', '&gt;');
text = strrep(text, '"', '&quot;');
text = strrep(text, '''', '&apos;');
end

function value = firstNumeric(value)
if iscell(value)
    value = value{1};
end
if isnumeric(value) || islogical(value)
    value = double(value(:));
    if isempty(value)
        value = NaN;
    else
        value = value(1);
    end
else
    value = NaN;
end
end

function root = defaultRepoRoot()
scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    root = pwd;
    return;
end
scriptDir = fileparts(scriptPath);
root = fileparts(scriptDir);
end

function tf = istableLike(value)
tf = istable(value) || istimetableLike(value);
end

function tf = istimetableLike(value)
tf = isa(value, 'timetable');
end

function tf = isContainerLike(value)
tf = istableLike(value) || isstruct(value) || isobject(value);
end

function tf = isTextScalar(value)
tf = ischar(value) || (isstring(value) && isscalar(value));
end

function tf = isEmptyText(value)
tf = isTextScalar(value) && strlength(string(value)) == 0;
end

function tf = isLogicalScalar(value)
tf = (islogical(value) || isnumeric(value)) && isscalar(value);
end

function tf = isNumericScalar(value)
tf = isnumeric(value) && isscalar(value);
end

function tf = isCellstrLike(value)
tf = iscell(value) && all(cellfun(@isTextScalar, value));
end

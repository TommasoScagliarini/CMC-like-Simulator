function summary = export_ab06_healthy_overlay(varargin)
%EXPORT_AB06_HEALTHY_OVERLAY Export AB06 raw IK/ID .mat data for plotter overlay.
%
% The plotter expects a bundle-local data/healthy directory containing one
% *Kinematics_q.sto and one *Actuation_force.sto file. AB06 raw files are
% MATLAB tables, so this exporter keeps the conversion in MATLAB.

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(scriptDir);

p = inputParser;
addParameter(p, 'Task', 'treadmill', @isTextScalar);
addParameter(p, 'Trial', 'treadmill_01_01', @isTextScalar);
addParameter(p, 'Subject', 'AB06', @isTextScalar);
addParameter(p, 'SourceRoot', fullfile(repoRoot, 'models', 'AB06-raw', '10_09_18'), @isTextScalar);
addParameter(p, 'OutputDir', fullfile(repoRoot, 'models', 'AB06_SEASEA_Threadmill', 'data', 'healthy'), @isTextScalar);
parse(p, varargin{:});
opts = p.Results;

task = char(opts.Task);
trial = char(opts.Trial);
subject = upper(char(opts.Subject));
sourceRoot = char(opts.SourceRoot);
outputDir = char(opts.OutputDir);

ikPath = fullfile(sourceRoot, task, 'ik', [trial '.mat']);
idPath = fullfile(sourceRoot, task, 'id', [trial '.mat']);

ikTable = loadDataTable(ikPath, 'IK');
idTable = loadDataTable(idPath, 'ID');

ikTime = numericColumn(ikTable, 'Header', ikPath);
idTime = numericColumn(idTable, 'Header', idPath);
validateTimeVectors(ikTime, idTime, ikPath, idPath);

[targetCoords, ikSources, idSources] = ab06SeaseaOverlayMappings();
ikData = extractColumns(ikTable, ikSources, ikPath);
idData = extractColumns(idTable, idSources, idPath);
idColumns = strcat('reserve_', targetCoords);

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

outputStem = [subject '_' sanitizeFileToken(trial)];
kinematicsPath = fullfile(outputDir, [outputStem '_Kinematics_q.sto']);
actuationPath = fullfile(outputDir, [outputStem '_Actuation_force.sto']);

writeSto(kinematicsPath, 'Coordinates', ikTime, targetCoords, ikData, true);
writeSto(actuationPath, 'ActuatorForces', ikTime, idColumns, idData, false);

summary = struct( ...
    'Subject', string(subject), ...
    'Task', string(task), ...
    'Trial', string(trial), ...
    'Rows', numel(ikTime), ...
    'StartTime', ikTime(1), ...
    'EndTime', ikTime(end), ...
    'KinematicsFile', string(kinematicsPath), ...
    'ActuationForceFile', string(actuationPath), ...
    'CoordinateCount', numel(targetCoords), ...
    'ActuationColumnCount', numel(idColumns));

fprintf('%s healthy overlay exported.\n', subject);
fprintf('  IK source: %s\n', ikPath);
fprintf('  ID source: %s\n', idPath);
fprintf('  Output dir: %s\n', outputDir);
fprintf('  Rows: %d\n', summary.Rows);
fprintf('  Time span: %.10g - %.10g s\n', summary.StartTime, summary.EndTime);
fprintf('  Kinematics columns: %d\n', summary.CoordinateCount);
fprintf('  Actuation columns: %d\n', summary.ActuationColumnCount);
fprintf('  Kinematics file: %s\n', kinematicsPath);
fprintf('  Actuation file: %s\n', actuationPath);

end

function tf = isTextScalar(value)
tf = ischar(value) || (isstring(value) && isscalar(value));
end

function token = sanitizeFileToken(value)
token = char(string(value));
token = regexprep(token, '[^A-Za-z0-9_-]+', '_');
end

function tbl = loadDataTable(path, label)
if ~isfile(path)
    error('%s source file not found: %s', label, path);
end
payload = load(path);
if ~isfield(payload, 'data')
    error('%s source file does not contain a data variable: %s', label, path);
end
tbl = payload.data;
if ~istable(tbl)
    error('%s data variable is not a table: %s', label, path);
end
end

function values = numericColumn(tbl, name, path)
if ~ismember(name, tbl.Properties.VariableNames)
    error('Missing column %s in %s', name, path);
end
values = double(tbl.(name));
if ~isvector(values) || isempty(values)
    error('Column %s in %s is empty or not a vector.', name, path);
end
values = values(:);
end

function validateTimeVectors(ikTime, idTime, ikPath, idPath)
if numel(ikTime) ~= numel(idTime)
    error( ...
        'IK/ID row count mismatch: %s has %d rows, %s has %d rows.', ...
        ikPath, numel(ikTime), idPath, numel(idTime));
end
maxDelta = max(abs(ikTime - idTime));
if maxDelta > 1e-9
    error('IK/ID time vectors differ; max delta is %.12g s.', maxDelta);
end
if any(diff(ikTime) <= 0)
    error('IK time vector is not strictly increasing: %s', ikPath);
end
end

function matrix = extractColumns(tbl, names, path)
nRows = height(tbl);
matrix = zeros(nRows, numel(names));
for iName = 1:numel(names)
    name = names{iName};
    if ~ismember(name, tbl.Properties.VariableNames)
        error('Missing column %s in %s', name, path);
    end
    values = double(tbl.(name));
    if numel(values) ~= nRows
        error('Column %s in %s has %d rows, expected %d.', name, path, numel(values), nRows);
    end
    matrix(:, iName) = values(:);
end
end

function [targetCoords, ikSources, idSources] = ab06SeaseaOverlayMappings()
targetCoords = { ...
    'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', ...
    'pelvis_tx', 'pelvis_ty', 'pelvis_tz', ...
    'hip_flexion_r', 'hip_adduction_r', 'hip_rotation_r', ...
    'knee_angle_r', 'ankle_angle_r', 'subtalar_angle_r', 'mtp_angle_r', ...
    'hip_flexion_l', 'hip_adduction_l', 'hip_rotation_l', ...
    'pros_knee_angle', 'pros_ankle_angle', ...
    'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'};

ikSources = targetCoords;
ikSources(strcmp(targetCoords, 'pros_knee_angle')) = {'knee_angle_l'};
ikSources(strcmp(targetCoords, 'pros_ankle_angle')) = {'ankle_angle_l'};

idSources = { ...
    'pelvis_tilt_moment', 'pelvis_list_moment', 'pelvis_rotation_moment', ...
    'pelvis_tx_force', 'pelvis_ty_force', 'pelvis_tz_force', ...
    'hip_flexion_r_moment', 'hip_adduction_r_moment', 'hip_rotation_r_moment', ...
    'knee_angle_r_moment', 'ankle_angle_r_moment', 'subtalar_angle_r_moment', 'mtp_angle_r_moment', ...
    'hip_flexion_l_moment', 'hip_adduction_l_moment', 'hip_rotation_l_moment', ...
    'knee_angle_l_moment', 'ankle_angle_l_moment', ...
    'lumbar_extension_moment', 'lumbar_bending_moment', 'lumbar_rotation_moment'};
end

function writeSto(path, headerName, time, colNames, data, inDegrees)
fid = fopen(path, 'w');
if fid < 0
    error('Could not open output file for writing: %s', path);
end
cleanup = onCleanup(@() fclose(fid));

if inDegrees
    degreeText = 'yes';
else
    degreeText = 'no';
end

fprintf(fid, '%s\n', headerName);
fprintf(fid, 'version=1\n');
fprintf(fid, 'nRows=%d\n', numel(time));
fprintf(fid, 'nColumns=%d\n', numel(colNames) + 1);
fprintf(fid, 'inDegrees=%s\n\n', degreeText);
fprintf(fid, 'Units are S.I. units (second, meters, Newtons, ...)\n');
if inDegrees
    fprintf(fid, 'Angles are in degrees.\n\n');
else
    fprintf(fid, 'Generalized forces are in S.I. units.\n\n');
end
fprintf(fid, 'endheader\n');
fprintf(fid, 'time');
for iCol = 1:numel(colNames)
    fprintf(fid, '\t%s', colNames{iCol});
end
fprintf(fid, '\n');

for iRow = 1:numel(time)
    fprintf(fid, '%.10f', time(iRow));
    fprintf(fid, '\t%.10f', data(iRow, :));
    fprintf(fid, '\n');
end
end

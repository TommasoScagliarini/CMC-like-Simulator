function result = test_convert_epic_time_header(repoRoot)
% Production converter checks on isolated synthetic MATLAB payloads.
% No OpenSim, model preparation or simulation is executed.
if nargin == 0
    repoRoot = fileparts(fileparts(mfilename('fullpath')));
end
addpath(fullfile(repoRoot, 'tools'));
fixtureRoot = tempname;
mkdir(fixtureRoot);
cleanup = onCleanup(@() rmdir(fixtureRoot, 's')); %#ok<NASGU>
sourceRoot = fullfile(fixtureRoot, 'source');
outputRoot = fullfile(fixtureRoot, 'output');
times = (10.36:0.005:10.38).';
condition = [10.359, 10.38];

writeFixture(sourceRoot, times, condition, 'header', times);
summary = convertFixture(repoRoot, sourceRoot, outputRoot);
assert(max(abs(readTrcTimes(summary.TrcFile) - times)) < 1e-10);
assert(max(abs(readMotTimes(summary.DatasetIkMotFile) - times)) < 1e-10);
setup = fileread(summary.IkSetupFile);
range = sscanf(extractBetween(string(setup), '<time_range>', '</time_range>'), '%f');
assert(max(abs(range(:) - [times(1); times(end)])) < 1e-10);
checks = 1;

% Real EPIC payloads are tables with Header as an ordinary numeric column.
writeFixture(sourceRoot, times, condition, 'header_table', times);
summary = convertFixture(repoRoot, sourceRoot, outputRoot);
assert(max(abs(readTrcTimes(summary.TrcFile) - times)) < 1e-10);
assert(max(abs(readMotTimes(summary.DatasetIkMotFile) - times)) < 1e-10);
checks = checks + 1;

% Static Header retains 200 Hz even though the converter default is 100 Hz.
staticTime = (0:0.005:0.02).';
writeFixture(sourceRoot, staticTime, [NaN, NaN], 'header', staticTime);
summary = convertFixture(repoRoot, sourceRoot, outputRoot);
assert(max(abs(readTrcTimes(summary.TrcFile) - staticTime)) < 1e-10);
assert(max(abs(readMotTimes(summary.DatasetIkMotFile) - staticTime)) < 1e-10);
checks = checks + 1;

% Legacy inputs without Header still use the existing conditions fallback.
writeFixture(sourceRoot, times, condition, 'absent', []);
summary = convertFixture(repoRoot, sourceRoot, outputRoot);
assert(max(abs(readTrcTimes(summary.TrcFile) - linspace(condition(1), condition(2), 5).')) < 1e-10);
checks = checks + 1;

% A present but malformed Header must never fall through to reconstructed time.
badHeaders = {[], times(1:4), [times(1:4); NaN], [times(1:4); Inf], ...
              [times(1:4); times(4)], flipud(times)};
for i = 1:numel(badHeaders)
    writeFixture(sourceRoot, times, condition, 'header', badHeaders{i});
    expectFailure(@() convertFixture(repoRoot, sourceRoot, outputRoot), ...
        'convert_epic_ab06_to_opensim:invalidTimeVector');
    checks = checks + 1;
end

writeFixture(sourceRoot, times, [20, 21], 'header', times);
expectFailure(@() convertFixture(repoRoot, sourceRoot, outputRoot), ...
    'convert_epic_ab06_to_opensim:emptyTimeRange');
checks = checks + 1;
result = struct('checks_passed', checks, 'uses_production_converter', true, ...
                'opensim_executed', false);
fprintf('Header time converter regression: %d/%d PASS\n', checks, checks);
end

function summary = convertFixture(repoRoot, sourceRoot, outputRoot)
summary = convert_epic_ab06_to_opensim('SubjectDir', sourceRoot, ...
    'Task', 'static', 'Trial', 'fixture', 'OutputDir', outputRoot, ...
    'TargetModel', 'AB07_SEASEA', ...
    'ModelFile', fullfile(repoRoot, 'models', 'AB06_SEASEA_Threadmill', 'AB06_SEASEA_stiff321_500_pi.osim'));
assert(summary.Status == "ok");
end

function writeFixture(sourceRoot, time, condition, mode, header)
markerNames = {'R_ASIS','L_ASIS','L_PSIS','R_Thigh_Upper','R_Thigh_Front', ...
    'R_Thigh_Rear','R_Knee_Lat','R_Shank_Upper','R_Shank_Front','R_Shank_Rear', ...
    'R_Ankle_Lat','R_Heel','R_Toe_Lat','R_Toe_Med','R_Toe_Tip','L_Thigh_Upper', ...
    'L_Thigh_Front','L_Thigh_Rear','L_Knee_Lat','L_Shank_Upper','L_Shank_Front', ...
    'L_Shank_Rear','L_Ankle_Lat','L_Heel','L_Toe_Lat','L_Toe_Med','L_Toe_Tip','R_PSIS'};
coordinateNames = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx', ...
    'pelvis_ty','pelvis_tz','hip_flexion_r','hip_adduction_r','hip_rotation_r', ...
    'knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r','hip_flexion_l', ...
    'hip_adduction_l','hip_rotation_l','knee_angle_l','ankle_angle_l', ...
    'lumbar_extension','lumbar_bending','lumbar_rotation'};
markers = struct();
ik = struct();
for i = 1:numel(markerNames)
    markers.(markerNames{i}) = repmat([0.1, 1, 0.1], numel(time), 1);
end
for i = 1:numel(coordinateNames)
    ik.(coordinateNames{i}) = zeros(numel(time), 1);
end
if startsWith(mode, 'header')
    % Deliberately conflicting legacy time columns prove Header precedence.
    markers.time = linspace(condition(1), condition(2), numel(time)).';
    ik.time = markers.time;
    markers.Header = header;
    ik.Header = header;
end
if strcmp(mode, 'header_table')
    markers = struct2table(markers);
    ik = struct2table(ik);
end
for name = {'markers', 'ik', 'conditions'}
    folder = fullfile(sourceRoot, 'static', name{1});
    if ~isfolder(folder), mkdir(folder); end
end
save(fullfile(sourceRoot, 'static', 'markers', 'fixture.mat'), 'markers');
save(fullfile(sourceRoot, 'static', 'ik', 'fixture.mat'), 'ik');
trialStarts = condition(1); %#ok<NASGU>
trialEnds = condition(2); %#ok<NASGU>
save(fullfile(sourceRoot, 'static', 'conditions', 'fixture.mat'), 'trialStarts', 'trialEnds');
end

function times = readTrcTimes(path)
lines = splitlines(string(fileread(path)));
times = [];
for i = 6:numel(lines)
    values = sscanf(lines(i), '%f');
    if numel(values) > 2
        times(end+1, 1) = values(2); %#ok<AGROW>
    end
end
end

function times = readMotTimes(path)
lines = splitlines(string(fileread(path)));
start = find(strcmpi(strtrim(lines), 'endheader'), 1);
times = [];
for i = start+1:numel(lines)
    values = sscanf(lines(i), '%f');
    if numel(values) > 2
        times(end+1, 1) = values(1); %#ok<AGROW>
    end
end
end

function expectFailure(callback, identifier)
try
    callback();
catch failure
    assert(strcmp(failure.identifier, identifier), ...
        'Unexpected failure %s: %s', failure.identifier, failure.message);
    return;
end
error('Expected failure %s did not occur.', identifier);
end

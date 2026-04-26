function data = loadVehicleData(filename)
%LOADVEHICLEDATA Professional loader for vehicle dynamics CSV files.
%
%   DATA = LOADVEHICLEDATA(FILENAME) reads a CSV file and returns a table.
%   Validates existence and basic structure before returning.

if nargin < 1 || isempty(filename)
    error('vdt:loadVehicleData:MissingFilename', 'Please specify a filename.');
end

if ~isfile(filename)
    error('vdt:loadVehicleData:FileNotFound', 'The file "%s" was not found.', filename);
end

try
    % Detect import options to handle different delimiters automatically
    opts = detectImportOptions(filename);
    data = readtable(filename, opts);
catch ME
    error('vdt:loadVehicleData:ImportFailed', ...
        'Failed to read CSV. MATLAB Error: %s', ME.message);
end

if isempty(data)
    error('vdt:loadVehicleData:EmptyFile', 'The loaded file is empty.');
end

% Standardize column names (convert to lowercase and remove spaces)
data.Properties.VariableNames = lower(strrep(data.Properties.VariableNames, ' ', '_'));

% Run logical validation
try
    validateVehicleData(data);
catch ME
    warning('vdt:loadVehicleData:ValidationWarning', ...
        'Data loaded but validation failed: %s', ME.message);
end

end

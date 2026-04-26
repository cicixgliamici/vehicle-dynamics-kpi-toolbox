function data = loadVehicleData(filename)
%LOADVEHICLEDATA Load vehicle dynamics data from a CSV file.

if ~isfile(filename)
    error("loadVehicleData:FileNotFound", "File not found: %s", filename);
end

data = readtable(filename);
validateVehicleData(data);
end

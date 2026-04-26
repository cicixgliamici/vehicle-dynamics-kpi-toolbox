% runBatchAnalysis.m
% Process all CSV files in the data/synthetic folder and export a summary.

clear; clc; close all;

% Setup paths
run(fullfile('..', 'startup.m'));

cfg = default_config();
dataDir = fullfile("..", "data", "synthetic");
resultsDir = fullfile("..", "results");

if ~exist(resultsDir, "dir"), mkdir(resultsDir); end

% Ensure data exists
if ~exist(dataDir, "dir")
    generateSyntheticData();
end

files = dir(fullfile(dataDir, "*.csv"));
allKpis = table();

for i = 1:numel(files)
    filename = files(i).name;
    fullPath = fullfile(dataDir, filename);
    
    fprintf("Processing %s...\n", filename);
    
    try
        rawData = loadVehicleData(fullPath);
        data = preprocessVehicleData(rawData, cfg);
        events = detectManeuvers(data, cfg);
        kpis = computeAllKPIs(data, events, cfg);
        
        % Add filename for tracking
        kpis.Dataset = string(filename);
        kpis = movevars(kpis, "Dataset", "Before", 1);
        
        allKpis = [allKpis; kpis];
    catch ME
        fprintf("Error processing %s: %s\n", filename, ME.message);
    end
end

disp("Batch processing complete.");
disp(allKpis);

exportKpiSummary(allKpis, fullfile(resultsDir, "kpi_summary.csv"));

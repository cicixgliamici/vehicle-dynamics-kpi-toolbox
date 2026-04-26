% runBatchAnalysis.m
% Run the full KPI pipeline on all synthetic datasets.

clear; clc; close all;
addpath(genpath(pwd));

cfg = default_config();

if ~isfile(fullfile("data", "synthetic", "step_steer.csv"))
    generateSyntheticData();
end

files = dir(fullfile("data", "synthetic", "*.csv"));
allRows = table();

for i = 1:numel(files)
    inputFile = fullfile(files(i).folder, files(i).name);
    fprintf("Processing %s\n", inputFile);

    rawData = loadVehicleData(inputFile);
    data = preprocessVehicleData(rawData, cfg);
    events = detectManeuvers(data, cfg);
    kpis = computeAllKPIs(data, events, cfg);

    kpis.dataset = string(files(i).name);
    allRows = [allRows; kpis]; %#ok<AGROW>
end

if ~exist("reports", "dir")
    mkdir("reports");
end

writetable(allRows, fullfile("reports", "batch_kpi_summary.csv"));
fprintf("Batch report exported to reports/batch_kpi_summary.csv\n");

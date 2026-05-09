% runSingleManeuverAnalysis.m
% Example script for processing one selected maneuver.

clear; clc; close all;

% Setup paths
run(fullfile('..', 'startup.m'));

cfg = default_config();

% Ensure data exists
if ~isfile(fullfile("..", "data", "synthetic", "sine_steer.csv"))
    generateSyntheticData();
end

inputFile = fullfile("..", "data", "synthetic", "sine_steer.csv");

rawData = loadVehicleData(inputFile);
data = preprocessVehicleData(rawData, cfg);

% Apply Bicycle Model
[theory, modelKpis] = simulateBicycleModel(data, cfg);
data = [data theory]; % Append theoretical signals to main table

events = detectManeuvers(data, cfg);
[kpis, frResults] = computeAllKPIs(data, events, cfg);

disp("Computed KPIs:");
disp(kpis);
disp("Model Theoretical KPIs:");
disp(modelKpis);

% Plotting
figuresDir = fullfile("..", "figures");
plotResults(data, kpis, figuresDir, frResults);

fprintf("Analysis complete.\n");

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
kpis = computeAllKPIs(data, events, cfg);

disp("Computed KPIs:");
disp(kpis);
disp("Model Theoretical KPIs:");
disp(modelKpis);

% Plotting
figuresDir = fullfile("..", "figures");
if ~exist(figuresDir, "dir"), mkdir(figuresDir); end

plotTimeSeries(data, fullfile(figuresDir, "sine_steer_timeseries.png"));
plotYawResponse(data, kpis, fullfile(figuresDir, "sine_steer_yaw_response.png"));

fprintf("Analysis complete. Figures saved in %s\n", figuresDir);

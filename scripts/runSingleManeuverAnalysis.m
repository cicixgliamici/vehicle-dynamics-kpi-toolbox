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

% Apply Bicycle Model (steady-state ground truth)
[theory, modelKpis] = simulateBicycleModel(data, cfg);
data = [data theory]; % Append theoretical signals to main table

events = detectManeuvers(data, cfg);
[kpis, frResults] = computeAllKPIs(data, events, cfg);

% --- Kalman Filter: Sideslip Angle Estimation ---
% Estimates the vehicle sideslip angle β, which is not directly measurable
% with standard sensors. Uses a 2-state linear Kalman Filter derived from
% the bicycle model.
[kfResults, kf_kpis] = estimateSideslip(data, cfg);

disp("Computed KPIs:");
disp(kpis);
disp("Bicycle Model KPIs:");
disp(modelKpis);
disp("Kalman Filter Sideslip KPIs:");
disp(kf_kpis);

% Plotting
figuresDir = fullfile("..", "figures");
plotResults(data, kpis, figuresDir, frResults);
plotSideslipEstimate(data, kfResults, fullfile(figuresDir, 'sideslip_kf.png'));

fprintf("Analysis complete. Figures saved to: %s\n", figuresDir);


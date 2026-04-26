% runSingleManeuverAnalysis.m
% Example script for processing one selected maneuver.

clear; clc; close all;
addpath(genpath(pwd));

cfg = default_config();
inputFile = fullfile("data", "synthetic", "sine_steer.csv");

rawData = loadVehicleData(inputFile);
data = preprocessVehicleData(rawData, cfg);
events = detectManeuvers(data, cfg);
kpis = computeAllKPIs(data, events, cfg);

disp(kpis);
plotTimeSeries(data, fullfile("figures", "sine_steer_timeseries.png"));

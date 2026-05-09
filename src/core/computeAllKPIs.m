function [kpis, frResults] = computeAllKPIs(data, events, cfg)
%COMPUTEALLKPIS Orchestrator to compute all KPI families and combine results.
%
%   This function calls the specialized sub-modules and merges their 
%   tables into a single, comprehensive row of metrics.

% 1. Extract Metrics by Family
handling = computeHandlingKPIs(data, events, cfg);
steering = computeSteeringKPIs(data, events, cfg);
ride = computeRideKPIs(data, events, cfg);

% 2. Frequency Analysis
if height(data) > cfg.freq.windowLength * 2
    [frResults, frMetrics] = computeFrequencyResponse(data, cfg);
else
    frResults = table();
    frMetrics = table();
end

% 3. Merge logic
% Add Steering KPIs (only the new columns)
newSteeringCols = setdiff(steering.Properties.VariableNames, handling.Properties.VariableNames);
kpis = [handling steering(:, newSteeringCols)];

% Add Ride KPIs (only the new columns)
existingCols = kpis.Properties.VariableNames;
newRideCols = setdiff(ride.Properties.VariableNames, existingCols);
kpis = [kpis ride(:, newRideCols)];

% Add Frequency Metrics
if ~isempty(frMetrics)
    existingCols = kpis.Properties.VariableNames;
    newFreqCols = setdiff(frMetrics.Properties.VariableNames, existingCols);
    kpis = [kpis frMetrics(:, newFreqCols)];
end

end

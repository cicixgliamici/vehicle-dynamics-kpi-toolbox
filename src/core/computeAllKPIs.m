function kpis = computeAllKPIs(data, events, cfg)
%COMPUTEALLKPIS Orchestrator to compute all KPI families and combine results.
%
%   This function calls the specialized sub-modules and merges their 
%   tables into a single, comprehensive row of metrics.

% 1. Extract Metrics by Family
% Each function returns a table with its specific parameters
handling = computeHandlingKPIs(data, events, cfg);
steering = computeSteeringKPIs(data, events, cfg);
ride = computeRideKPIs(data, events, cfg);

% 2. Vertical Merge logic
% We use horizontal concatenation [A B C].
% To avoid duplicate column errors (like 'time_s' if present), we filter 
% out names that are already in the previous tables.

% Add Steering KPIs (only the new columns)
newSteeringCols = setdiff(steering.Properties.VariableNames, handling.Properties.VariableNames);
kpis = [handling steering(:, newSteeringCols)];

% Add Ride KPIs (only the new columns)
existingCols = kpis.Properties.VariableNames;
newRideCols = setdiff(ride.Properties.VariableNames, existingCols);
kpis = [kpis ride(:, newRideCols)];

% Result is a 1xN table ready for export or display
end

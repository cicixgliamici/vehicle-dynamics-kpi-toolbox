function kpis = computeAllKPIs(data, events, cfg)
%COMPUTEALLKPIS Compute all KPI families and combine them into one row.

handling = computeHandlingKPIs(data, events, cfg);
steering = computeSteeringKPIs(data, events, cfg);
ride = computeRideKPIs(data, events, cfg);

kpis = [handling steering(:, setdiff(steering.Properties.VariableNames, handling.Properties.VariableNames)) ...
        ride(:, setdiff(ride.Properties.VariableNames, [handling.Properties.VariableNames steering.Properties.VariableNames]))];
end

function plotResults(data, kpis, outputDir, frResults)
%PLOTRESULTS High-level orchestrator to generate and save all standard plots.
%
%   plotResults(data, kpis, outputDir, frResults) generates time-series,
%   handling characteristic, and frequency response plots.

if nargin < 3
    outputDir = '.';
end

if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% 1. Time Series Plot
plotTimeSeries(data, fullfile(outputDir, 'timeseries.png'));

% 2. Yaw Response Plot
plotYawResponse(data, kpis, fullfile(outputDir, 'yaw_response.png'));

% 3. Frequency Response Plot
if nargin > 3 && ~isempty(frResults)
    plotFrequencyResponse(frResults, fullfile(outputDir, 'frequency_response.png'));
end

fprintf('Plots generated and saved to: %s\n', outputDir);

end

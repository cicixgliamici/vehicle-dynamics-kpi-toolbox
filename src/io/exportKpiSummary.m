function exportKpiSummary(kpis, outputFile, sourceFile)
%EXPORTKPISUMMARY Export KPI results to a structured CSV with metadata.
%
%   exportKpiSummary(kpis, outputFile) saves a KPI table to CSV with a
%   timestamp and toolbox version prepended for traceability.
%
%   exportKpiSummary(kpis, outputFile, sourceFile) also records the name
%   of the original data file that produced the KPIs.
%
%   Inputs:
%       kpis       - 1xN Table of computed KPIs (one row per maneuver).
%       outputFile - Full path of the output CSV file (e.g., 'results/kpi_summary.csv').
%       sourceFile - (Optional) Original data filename for audit trail.
%
%   Example:
%       exportKpiSummary(kpis, 'results/kpi_summary.csv', 'data/sine_steer.csv');
%
%   See also: COMPUTEALLKPIS, WRITETABLE

if nargin < 2 || isempty(outputFile)
    error('vdt:exportKpiSummary:MissingOutput', 'Please specify an output file path.');
end
if nargin < 3 || isempty(sourceFile)
    sourceFile = 'unknown';
end
if isempty(kpis)
    warning('vdt:exportKpiSummary:EmptyKPIs', 'KPI table is empty. Nothing was written.');
    return;
end

% Ensure output directory exists
outputDir = fileparts(outputFile);
if ~isempty(outputDir) && ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Build metadata prefix columns
metaTable = table( ...
    string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    string(sourceFile), ...
    'VariableNames', {'export_timestamp', 'source_file'});

% Merge metadata + KPIs into a single flat report row
fullReport = [metaTable, kpis];

% Append to existing file (for batch runs) or create a new one
if isfile(outputFile)
    % Read existing header to check compatibility
    existingReport = readtable(outputFile, 'VariableNamingRule', 'preserve');
    if isequal(existingReport.Properties.VariableNames, fullReport.Properties.VariableNames)
        fullReport = [existingReport; fullReport];
    else
        warning('vdt:exportKpiSummary:SchemaMismatch', ...
            'Existing file has different columns. Overwriting: %s', outputFile);
    end
end

writetable(fullReport, outputFile);
fprintf('KPI summary saved to: %s (%d rows total)\n', outputFile, height(fullReport));

end

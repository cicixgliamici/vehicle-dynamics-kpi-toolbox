function exportKpiSummary(kpis, filename)
%EXPORTKPISUMMARY Export KPI table to CSV or Excel.

if isempty(kpis)
    warning("exportKpiSummary:EmptyTable", "KPI table is empty.");
    return;
end

writetable(kpis, filename);
fprintf("KPI summary exported to %s\n", filename);

end

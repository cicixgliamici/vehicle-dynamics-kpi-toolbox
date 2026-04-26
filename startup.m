% startup.m
% Run this script to add all necessary folders to the MATLAB path.

projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

% Remove hidden folders from path (like .git)
p = split(path, pathsep);
p = p(~contains(p, '.git'));
path(strjoin(p, pathsep));

fprintf('Vehicle Dynamics KPI Toolbox initialized.\n');
fprintf('Project root: %s\n', projectRoot);

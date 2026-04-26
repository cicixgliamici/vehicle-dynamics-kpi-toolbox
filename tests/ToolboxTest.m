classdef ToolboxTest < matlab.unittest.TestCase
    % TOOLBOXTEST Professional unit tests for the Vehicle Dynamics KPI Toolbox.
    
    properties
        TestData
        Config
    end
    
    methods(TestClassSetup)
        function setupPath(testCase)
            % Ensure all source folders are on the path during CI
            here = fileparts(mfilename('fullpath'));
            projectRoot = fullfile(here, '..');
            addpath(genpath(projectRoot));
        end
    end

    methods(TestClassTeardown)
        function cleanupPath(testCase)
            % Clean up the path after tests (Production best practice)
            here = fileparts(mfilename('fullpath'));
            projectRoot = fullfile(here, '..');
            rmpath(genpath(projectRoot));
        end
    end

    methods(TestMethodSetup)
        function setupData(testCase)
            % This runs before EACH test method
            testCase.Config = default_config();
            
            % Create a clean ramp steering signal
            t = (0:0.01:10)';
            steer = zeros(size(t));
            steer(t > 2) = 20 * (t(t > 2) - 2); 
            steer(steer > 50) = 50;
            
            % Simple response: Gain = 0.5
            yaw = 0.5 * steer;
            latacc = 0.2 * yaw;
            
            % Use cellstr for maximum compatibility with older MATLAB versions
            varNames = cellstr(testCase.Config.requiredColumns);
            
            testCase.TestData = table(t, 20*ones(size(t)), steer, yaw, latacc, ...
                zeros(size(t)), zeros(size(t)), zeros(size(t)), ...
                'VariableNames', varNames);
        end
    end
    
    methods(Test)
        function testFiltering(testCase)
            % Test if the low-pass filter actually reduces noise
            signal = randn(1000, 1);
            origStd = std(signal);
            
            filtered = lowPassFilterSignal(signal, 10);
            filtStd = std(filtered);
            
            testCase.verifyLessThan(filtStd, origStd, 'Filter should reduce signal variance');
        end
        
        function testManeuverDetection(testCase)
            % Test if we detect at least one event in our ramp steering
            events = detectManeuvers(testCase.TestData, testCase.Config);
            
            testCase.verifyNotEmpty(events, 'Should detect a steering event');
            testCase.verifyEqual(events.type(1), "steering_event", 'Event type mismatch');
        end
        
        function testKpiCalculation(testCase)
            % Test if gains are calculated correctly
            events = detectManeuvers(testCase.TestData, testCase.Config);
            kpis = computeAllKPIs(testCase.TestData, events, testCase.Config);
            
            % In our setup, yaw = 0.5 * steer -> Gain should be approx 0.5
            testCase.verifyGreaterThan(kpis.yawRateGain_1ps, 0.4, 'Gain calculation inaccurate');
            testCase.verifyLessThan(kpis.yawRateGain_1ps, 0.6, 'Gain calculation inaccurate');
        end

        function testBicycleModel(testCase)
            % Test if the model produces the expected understeer gradient
            [~, modelKpis] = simulateBicycleModel(testCase.TestData, testCase.Config);
            
            testCase.verifyTrue(isnumeric(modelKpis.Kus_deg_g), 'Model KPI should be numeric');
            testCase.verifyGreaterThan(modelKpis.Kus_deg_g, 0, 'Standard vehicle should be understeering');
        end

        % --- Production Readiness (Edge Cases) ---

        function testEmptyDataError(testCase)
            % Verify that functions throw correct error IDs for empty input
            emptyTab = table();
            testCase.verifyError(@() computeHandlingKPIs(emptyTab, table(), testCase.Config), ...
                'vdt:computeHandlingKPIs:InvalidData');
        end

        function testMissingColumnError(testCase)
            % Verify validation detects missing columns
            badData = testCase.TestData;
            badData.yaw_rate_degps = [];
            testCase.verifyError(@() validateVehicleData(badData), ...
                'validateVehicleData:MissingColumns');
        end

        function testNaNRobustness(testCase)
            % Verify that removeMissingSamples handles NaNs correctly
            noisyData = testCase.TestData;
            noisyData.yaw_rate_degps(5:10) = NaN;
            
            cleaned = removeMissingSamples(noisyData);
            testCase.verifyFalse(any(isnan(cleaned.yaw_rate_degps)), 'NaNs should be interpolated');
        end
    end
end

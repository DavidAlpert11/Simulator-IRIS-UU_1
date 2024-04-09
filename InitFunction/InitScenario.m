function ScenarioParameter = InitScenario
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
ScenarioParameter.dt = 0.01;%[sec]
ScenarioParameter.FinalTime =10000000000;%[sec]
ScenarioParameter.g = 9.81;%[m/s^2]
ScenarioParameter.ScenarioMode = 1;%static
ScenarioParameter.GPS_freq = 1/ScenarioParameter.dt; 
ScenarioParameter.isGPSAvailable=1;
ScenarioParameter.IdealIMU =0;
end


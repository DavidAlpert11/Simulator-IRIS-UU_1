function [DynamicStateParameter] = InitDynamic(ScenarioParameter,State)
%UNTITLED16 Summary of this function goes here
%   Detailed explanation goes here
%todo
DynamicStateParameter.f_b = [0,0,0]';
DynamicStateParameter.w_b = zeros(3,1);
DynamicStateParameter.w_b_dot = zeros(3,1);
if (ScenarioParameter.ScenarioMode ==0)%static
    DynamicStateParameter.f_b = State.C_B2I'*[0,0,-ScenarioParameter.g]';
    
end
end


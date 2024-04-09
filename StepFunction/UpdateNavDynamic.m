function [DynamicNavStateParameter] = UpdateNavDynamic(SensorMeas,NavState)
%UNTITLED26 Summary of this function goes here
%   Detailed explanation goes here
DynamicNavStateParameter.f_b = SensorMeas.f - NavState.bias_acc;
DynamicNavStateParameter.w_b = SensorMeas.w - NavState.bias_gyro;
end


function State = UpdateTrueIMUBias(SensorMeas,DynamicStateParameter,State)
%UNTITLED27 Summary of this function goes here
%   Detailed explanation goes here
State.bias_acc = SensorMeas.f-DynamicStateParameter.f_b ;
State.bias_gyro = SensorMeas.w-DynamicStateParameter.w_b;
end


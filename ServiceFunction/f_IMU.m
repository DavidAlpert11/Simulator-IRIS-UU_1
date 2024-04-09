function [ f_IMU ] = f_IMU(f_true,IMUParameters)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
f_IMU = f_true+IMUParameters.b_a+IMUParameters.w_a*randn(3,1)+(IMUParameters.Ma+IMUParameters.Sa)*f_true;

end



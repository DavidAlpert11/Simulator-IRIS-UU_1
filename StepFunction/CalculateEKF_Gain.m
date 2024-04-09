function [K] = CalculateEKF_Gain(EKF)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
K = EKF.P*EKF.H'/(EKF.H*EKF.P*EKF.H'+EKF.R);  %   Kalman gain
end


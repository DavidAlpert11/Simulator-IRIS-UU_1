function [ w_IMU ] = w_IMU(w_true,bias,whiteNoise,Mg,Sg,f_true,G)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
w_IMU = w_true+bias+whiteNoise+(Mg+Sg)*w_true+G*f_true;

end
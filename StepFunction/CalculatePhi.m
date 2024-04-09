function [Phi] = CalculatePhi(C_B2I_estimate,f_b,dt,GM,G)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
F_aug = CalculateF(C_B2I_estimate,f_b,GM,G);

Phi = expm(F_aug*dt);
end


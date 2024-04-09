function [F_aug] = CalculateF(C_B2I_estimate,f_b,GM,G)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

% specific force
f_n = C_B2I_estimate*f_b;

% F matrix of error model
F = [ zeros(3),eye(3), zeros(3);
    zeros(3), zeros(3),skew( -f_n );
    zeros(3), zeros(3), zeros(3)];
%   F matrix of the augmented error state
F_aug = [   F           G   ;...
    zeros(6,9)  GM  ];
end


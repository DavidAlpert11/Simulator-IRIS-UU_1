function [Ggamma_Q_Gamma_t] = CalculateGammaQ(G,Q,dt)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
%    G matrix of the augmented error state
G_aug = [   G           zeros(9,6)  ;...
    zeros(6)    eye(6)      ];
Ggamma_Q_Gamma_t = G_aug*Q*G_aug'*dt;
end


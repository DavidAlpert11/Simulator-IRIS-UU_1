function [z] = rand_mu_sigma(mu,sigma,row,col)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
% R = chol(sigma);
% z = repmat(mu,row,col) + randn(row,col)*R;
z = normrnd(mu,sigma,row,col);
end


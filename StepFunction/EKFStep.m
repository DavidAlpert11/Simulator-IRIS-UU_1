function [NavState,EKF] = EKFStep(ScenarioParameter,NavState,State,DynamicNavStateParameter,EKF,i)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here

C_B2I_estimate = (eye(3)-skew([EKF.delta_x(7),EKF.delta_x(8),EKF.delta_x(9)]))*NavState.C_B2I;

%   Gyro bias process charectaristic time [sec]
G = [   zeros(3)    zeros(3)    ;...
    C_B2I_estimate       zeros(3)    ;...
    zeros(3)    C_B2I_estimate       ];

%   Transformation matrix
Phi = CalculatePhi(C_B2I_estimate,DynamicNavStateParameter.f_b,ScenarioParameter.dt,EKF.GM,G);
%   Update deltaX_minus 
EKF.delta_x = Phi*EKF.delta_x;

%   Process noise term calculation (Gamma*Q*Gamma'), assuming deltaT is small
Ggamma_Q_Gamma_t = CalculateGammaQ(G,EKF.Q,ScenarioParameter.dt);
%   Next P matrix calculation
EKF.P = Phi*EKF.P*Phi' + Ggamma_Q_Gamma_t;

% GPS Measurements
if (ScenarioParameter.ScenarioMode ==0)%static
    ScenarioParameter.isGPSAvailable=1;
end

if (ScenarioParameter.isGPSAvailable)
    if(mod(i,ScenarioParameter.GPS_freq) == 0)
        
        %     if (ScenarioParameter.isGPSAvailable)
        delta_Z = GetDeltaZ(EKF.H,NavState,State);
        %     else
        %             delta_Z = GetDeltaZ_2(EKF.H,NavState,State);
        %
        %     end
        %Calculate Kalman Filter gain
        K = CalculateEKF_Gain(EKF);
        
        %   Error state update
        
        EKF.delta_x = EKF.delta_x + K*(delta_Z-EKF.H*EKF.delta_x);
        EKF.P = EKF.P-K*EKF.H*EKF.P;
    end
else
    debug =1;
end
% if(mod(Quad.i,Quad.GPS_freq) == 0 && Quad.isGPSAvailable)
% UpdateNavStateFromEKF
NavState = UpdateNavStateFromEKF(EKF.delta_x,NavState,C_B2I_estimate);

% reset delta_x for close loop
EKF.delta_x(1:end) = 0;

end


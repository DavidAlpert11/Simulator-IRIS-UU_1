function [NavState,Command] = InitNavState(Command,State,ScenarioParameter)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here

 NavState =State;
NavState.SigmaPosInit = 1;%[m]
NavState.SigmaVelInit = 0.1;%[m/s]
NavState.SigmaEulerInit = 0.01;%[deg]
    return;

if (ScenarioParameter.IdealIMU)
    NavState =State;
    NavState.SigmaPosInit = 0;%[m]
    NavState.SigmaVelInit = 0;%[m/s]
    NavState.SigmaEulerInit = 0;%[deg]
    return;
end
NavState.SigmaPosInit = 1;%[m]
NavState.SigmaVelInit = 0.1;%[m/s]
NavState.SigmaEulerInit = 0.01;%[deg]

NavState.X = State.X+NavState.SigmaPosInit*randn(3,1);
NavState.X_dot = State.X_dot+NavState.SigmaVelInit*randn(3,1);
Euler = NavState.SigmaEulerInit*randn(3,1)*pi/180;

NavState.phi = State.phi+Euler(1);
NavState.theta = State.theta+Euler(2);
NavState.psi = State.psi+Euler(3);

C_I2B = Euler2Dcm( NavState.phi,NavState.theta,NavState.psi );
NavState.C_B2I= C_I2B';

NavState.bias_acc = [0,0,0]';
NavState.bias_gyro = [0,0,0]';
end


function [State,Command] = InitState(IMUParameters,Command,ScenarioMode)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
State.X = [0,0,0]';
State.X_dot = [0,0,0]';
State.phi = 0*pi/180;
State.theta = 0*pi/180;
State.psi = 0*pi/180;

C_I2B = Euler2Dcm( State.phi,State.theta,State.psi );
State.C_B2I= C_I2B';

State.bias_acc = IMUParameters.b_a;
State.bias_gyro = IMUParameters.b_g;

if (ScenarioMode==1)%use Command struct
    State.X = Command.Pose_des_GF(Command.index_Pose_des_GF,1:3)';
    State.psi = Command.Pose_des_GF(Command.index_Pose_des_GF,4);
    Command.index_Pose_des_GF = Command.index_Pose_des_GF+1;
    
    
    
    C_I2B = Euler2Dcm( State.phi,State.theta,State.psi );
    State.C_B2I= C_I2B';
end
end


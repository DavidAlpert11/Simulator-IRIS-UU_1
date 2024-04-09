% function [RecordState,RecordNavState,RecordEKF,RecordCommand,RecordScenarioParameter] = InitRecordParameter(State,NavState,EKF,Command,i)
%UNTITLED24 Summary of this function goes here
%   Detailed explanation goes here
RecordState.X(:,1) = State.X;
RecordState.X_dot(:,1) = State.X_dot;
RecordState.phi(:,1) = State.phi;
RecordState.theta(:,1)= State.theta;
RecordState.psi(:,1) = State.psi;
RecordState.bias_acc(:,1) = State.bias_acc;
RecordState.bias_gyro(:,1) = State.bias_gyro;


RecordNavState.X(:,1) = NavState.X;
RecordNavState.X_dot(:,1) = NavState.X_dot;
RecordNavState.phi(:,1) = NavState.phi;
RecordNavState.theta(:,1) = NavState.theta;
RecordNavState.psi(:,1) = NavState.psi;
RecordNavState.bias_acc(:,1) = NavState.bias_acc;
RecordNavState.bias_gyro(:,1) = NavState.bias_gyro;

RecordEKF.P_diag_sqrt(:,1) = sqrt(diag(EKF.P));
RecordCommand.PsiCommand(:,1) = Command.psi_des;
if (Command.index_Pose_des_GF > length(Command.Pose_des_GF(:,1)))

RecordCommand.Pose_des_GF(:,1) = Command.Pose_des_GF(length(Command.Pose_des_GF(:,1)),1:3)';
else
RecordCommand.Pose_des_GF(:,1) = Command.Pose_des_GF(Command.index_Pose_des_GF,1:3)';

end
RecordScenarioParameter.isGPSAvailable(:,1) = ScenarioParameter.isGPSAvailable;
% end
% function [RecordState,RecordNavState,RecordEKF] = RecordParameter(State,NavState,EKF,i,RecordState,RecordNavState,RecordEKF)
%UNTITLED24 Summary of this function goes here
%   Detailed explanation goes here
t(i) = time;

RecordState.X(:,i) = State.X;
RecordState.X_dot(:,i) = State.X_dot;
RecordState.phi(:,i) = State.phi;
RecordState.theta(:,i)= State.theta;
RecordState.psi(:,i) = State.psi;
RecordState.bias_acc(:,i) = State.bias_acc;
RecordState.bias_gyro(:,i) = State.bias_gyro;


RecordNavState.X(:,i) = NavState.X;
RecordNavState.X_dot(:,i) = NavState.X_dot;
RecordNavState.phi(:,i) = NavState.phi;
RecordNavState.theta(:,i) = NavState.theta;
RecordNavState.psi(:,i) = NavState.psi;
RecordNavState.bias_acc(:,i) = NavState.bias_acc;
RecordNavState.bias_gyro(:,i) = NavState.bias_gyro;

RecordEKF.P_diag_sqrt(:,i) = sqrt(diag(EKF.P));
RecordCommand.PsiCommand(:,i) = Command.psi_des;
if (Command.index_Pose_des_GF > length(Command.Pose_des_GF(:,1)))

RecordCommand.Pose_des_GF(:,i) = Command.Pose_des_GF(length(Command.Pose_des_GF(:,1)),1:3)';
else
    RecordCommand.Pose_des_GF(:,i) = Command.Pose_des_GF(Command.index_Pose_des_GF,1:3)';

end
RecordScenarioParameter.isGPSAvailable(:,i) = ScenarioParameter.isGPSAvailable;

% end


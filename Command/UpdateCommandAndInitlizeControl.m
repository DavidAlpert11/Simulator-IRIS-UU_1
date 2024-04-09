function [Command,Control]=UpdateCommandAndInitlizeControl(State,Command,Control,NavState,ScenarioMode,i,time_in_location_error)

if (Command.index_Pose_des_GF > length(Command.Pose_des_GF(:,1)))
    Command.Finish_pose_command = 1;
    return;
end
    errorPose = norm(NavState.X- Command.Pose_des_GF(Command.index_Pose_des_GF,1:3)');


conditionErrorPos = errorPose < Command.Tolerance_pose_des(1,1);


    errorPsi =  abs(NavState.psi-Command.Pose_des_GF(Command.index_Pose_des_GF,4));


conditionErrorPsi = errorPsi < Command.Tolerance_pose_des(1,2);

% if (conditionErrorPos && conditionErrorPsi) %Finsish current command
    if (conditionErrorPos) %Finsish current command

    Command.PoseSimulation(Command.index_Pose_des_GF,:) = [State.X',State.psi,State.theta,NavState.theta,i]; 

    L=length(Command.Pose_des_GF(:,1));
    if (Command.index_Pose_des_GF<L)
        Command.TimeInLocationError(Command.index_Pose_des_GF) = time_in_location_error;
        Command.index_Pose_des_GF = Command.index_Pose_des_GF+1;
        Control.init = 0;
        display([num2str(Command.index_Pose_des_GF/L*100),'%']);
%         display (sum(Command.POIChecked))
    else
        if (ScenarioMode==0)
            Command.Finish_pose_command=0;
        else
            Command.Finish_pose_command = 1;
            Command.TimeInLocationError(Command.index_Pose_des_GF) = time_in_location_error;
            
        end
    end
end

init = Command.Pose_des_GF(Command.index_Pose_des_GF-1,1:4);
goal = Command.Pose_des_GF(Command.index_Pose_des_GF,1:4);
desire_pose = init+(goal-init);
normCommand =norm(desire_pose(1:3)' - NavState.X);
if (normCommand>Control.LimitCommand)
    desire_pose(1:3) = (Control.LimitCommand/normCommand)*(desire_pose(1:3)' - NavState.X)+NavState.X;
    Control.init = 0;
end
Command.X_des_GF = desire_pose(1:3)';        
Command.psi_des = desire_pose(4);
end

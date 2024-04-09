% function Environment= CheckCollision(Environment,State,time)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
C = Environment.BridgeObstaclePoint-State.X';
normC = vecnorm(C')';
[aa,bb] = find(normC<=Environment.RadiusObstacle);
if (~isempty(aa))
    if (Environment.Collision<1)
        Environment.TimeOfCollision = time;
    end
    Environment.Collision = 1;
end
% end


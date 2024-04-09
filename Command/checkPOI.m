% function Command = checkPOI(State,Environment,Command)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

FOV =cos(deg2rad(0.5*94));
A = [cos(State.psi)*cos(State.theta),sin(State.psi)*cos(State.theta),-sin(State.theta)]';

B = Environment.TargetsBridge(Command.POIUnique,:)-State.X';
normB = (vecnorm(B'))';
POIChecked = abs(B*A)./(norm(A).*normB) > FOV;
[aa,bb] = find(POIChecked > Command.POIChecked);
if (~isempty(aa))
    [cc,dd] = find(normB(aa)<10);
    if (~isempty(cc))
        C = Environment.BridgeObstaclePoint-State.X';
        normC = vecnorm(C')';
        [ee,ff] = find(normC<=max(normB(aa(cc))'));
        obstacleCheck = (C(ee,1:3)./normC(ee))*(B(aa(cc),1:3)'./normB(aa(cc))');
        y = Environment.RadiusObstacle; x = normC(ee); 
        FOVObstacle = cos(atan2(y,x));
        [hh,ii] = find(obstacleCheck>FOVObstacle);
        jj = ones(size(unique(ii)))';
        Command.POIChecked(aa(cc(unique(ii))),bb(dd(ff(jj)))) = POIChecked(aa(cc(unique(ii))),bb(dd(ff(jj))));


        return;
    end
end

% end



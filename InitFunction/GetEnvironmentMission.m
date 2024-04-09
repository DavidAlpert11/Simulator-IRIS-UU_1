function [Environment]= GetEnvironmentMission(path_obj)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


% Environment.TargetsBridge = importdata([Rootfolder,'\TargetsBridge']);
% Environment.BridgeObstaclePoint = importdata([Rootfolder,'\BridgeObstaclePoint']);
% BridgeObj = readObj([Rootfolder,'bridge_small.obj']);
BridgeObj = readObj(path_obj);
% BridgeObj = readObj([Rootfolder,'High_Bridge.obj']);
% BridgeObj = readObj([Rootfolder,'simpleExample.obj']);


% BridgeObj = readObj([Rootfolder,'bridgeTestLarge.obj']);

% BridgeObj=read_wobj([Rootfolder,'bridge.obj'])
% BridgeObj = read_wobj([Rootfolder,'bridge.obj']);
Environment.obj = BridgeObj;

BridgeObjRaw = BridgeObj.v;
BridgeObjFaces = BridgeObj.f.v;
Environment.p0p1p2 = cell(1,1) ;   
for k=1:1:length(BridgeObjFaces)
        [p0p1p2] = BridgeObjRaw(BridgeObjFaces(k,:),:);
        Environment.p0p1p2{k} = p0p1p2;
end
Environment.BridgeObjRaw = BridgeObj.v;

Environment.p0 = BridgeObjRaw(BridgeObjFaces(:,1),:);
Environment.p1 = BridgeObjRaw(BridgeObjFaces(:,2),:);
Environment.p2 = BridgeObjRaw(BridgeObjFaces(:,3),:);

Environment.Collision = 0;
Environment.RadiusObstacle = 0.2;
Environment.TimeOfCollision=0;

TargetPoint = (zeros(1,3));
index =0;
for i=1:1:length(Environment.BridgeObjRaw(:,1))
    v = Environment.BridgeObjRaw(i,1:3);
    if i==1
        index = index +1;
        TargetPoint(index,:) = v;
    else
        [~,index_min] = min(vecnorm((TargetPoint-v)'));
%         if (norm(TargetPoint(index_min,:)-v)<1e-3)
        if (norm(TargetPoint(index_min,:)-v)<0.2)
            continue;
        end
        index = index +1;
        TargetPoint(index,:) = v;
    end
end
length(TargetPoint)
Environment.TargetsBridge = (TargetPoint);
end


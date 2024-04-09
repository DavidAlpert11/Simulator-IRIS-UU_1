function Command = InitCommand(FileName,Rootfolder)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%todo 
Command.TimeInLocationError = zeros(1,1);
% Rootfolder = [IRIS_build_folder,'\'];
folder = [Rootfolder,FileName];
position_vertex = (importdata([folder,'_conf']));
fullFileName = fullfile([folder,'_result']);
% result_file = readtable(fullFileName,'ReadRowNames',true);
result_file = readtable(fullFileName);

fid = fopen(fullFileName);
tline = fgets(fid);
while ischar(tline)
    disp(tline);
    prevLine = tline;
    tline = fgets(fid);
end
fclose(fid);
tempCell = split(prevLine,' ');
tempvalue = 0;
for i=1:1:length(tempCell)
    tempvalue(i) = str2double(tempCell{i});
end

    vertexTrajectory =tempvalue(2:end);
NumOfTotalVertices = tempvalue(1);

vertexTrajectoryTempForIndex = vertexTrajectory+1;
[aa,bb]=find(isnan(vertexTrajectoryTempForIndex)<1);
vertexTrajectoryTempForIndex = vertexTrajectoryTempForIndex(bb);
% position_vertex(vertexTrajectoryTempForIndex,4) = position_vertex(vertexTrajectoryTempForIndex,4)+position_vertex(vertexTrajectoryTempForIndex,5);
% position_vertex(vertexTrajectoryTempForIndex,4) = position_vertex(vertexTrajectoryTempForIndex,5);
Command.index_Pose_des_GF = 1;
if (max(vertexTrajectoryTempForIndex)<2) % no solution found
Command.Pose_des_GF = position_vertex(1,1:5);%x,y,z,psi
Command.X_des_GF = Command.Pose_des_GF(1,1:3)';         % desired value of X in Global frame
Command.psi_des = Command.Pose_des_GF(1,4);
else
Command.Pose_des_GF = position_vertex(vertexTrajectoryTempForIndex,1:5);%x,y,z,psi
Command.X_des_GF = Command.Pose_des_GF(Command.index_Pose_des_GF+1,1:3)';         % desired value of X in Global frame
Command.psi_des = Command.Pose_des_GF(Command.index_Pose_des_GF+1,4);
end


fid = fopen([folder,'_vertex']);
tline = fgetl(fid);
vertexCell = cell(1,1);
vertexCell{1} = tline;
index = 0;
POI = 0;
indexPOI = 0;
POICell = cell(1,1);
while ischar(tline)
    [aa,bb] = find(abs(vertexTrajectory-index)<0.5);
    if (isempty(aa))
        index=index+1;
        tline = fgetl(fid);
        continue
    end
    
    index=index+1;
    vertexCell{index} = tline;
    aa =str2num(vertexCell{index});
    for j =4:1:length(aa)
        indexPOI = indexPOI+1;
        POI(indexPOI) = aa(j);
    end
    if (length(aa)>3)
        POICell{index} = aa(4:end);
    else
        POICell{index} =[];
    end
    tline = fgetl(fid);
end
fclose(fid);
Command.POIUnique =  unique(POI)+1;
Command.POIChecked = zeros(size(Command.POIUnique))';
Command.POICell = POICell;
Command.vertexTrajectoryTempForIndex = vertexTrajectoryTempForIndex;

Command.Tolerance_pose_des = [0.5,1*pi/180];
Command.Finish_pose_command = 0;

%%
POIINDEX = 0;
index = 0;
tempCell = cell(1,1);
for i = 1 : 1:length(Command.vertexTrajectoryTempForIndex)
    for j = 1:1:length(Command.POICell{Command.vertexTrajectoryTempForIndex(i)})
        index = index+1;
        POIINDEX(index) = Command.POICell{Command.vertexTrajectoryTempForIndex(i)}(j);
    end
    tempCell{i} = Command.POICell{Command.vertexTrajectoryTempForIndex(i)};
    
end
% length(POIINDEX)
% length(unique(POIINDEX))
Command.POICellCommand = tempCell;
Command.POIINDEX = POIINDEX;



Command.VerticsPosition = position_vertex(1:NumOfTotalVertices,1:5);%x,y,z,psi

fid = fopen([folder,'_vertex']);
tline = fgetl(fid);
vertexCell = cell(1,1);
vertexCell{1} = tline;
index = 0;
POI = 0;
indexPOI = 0;
POICell = cell(1,1);
while ischar(tline)
    
    index=index+1;
    vertexCell{index} = tline;
    aa =str2num(vertexCell{index});
    for j =4:1:length(aa)
        indexPOI = indexPOI+1;
        POI(indexPOI) = aa(j);
    end
    if (length(aa)>3)
        POICell{index} = aa(4:end);
    else
        POICell{index} =[];
    end
    tline = fgetl(fid);
end
fclose(fid);
Command.POICellVertices = POICell;
end

function [POICell,collisionCheck] = GetInspectedPOIandCollision(Rootfolder)
nameOfFile = ['\test',num2str(1)];
folder = Rootfolder;%[Rootfolder,nameOfFile];
% fid = fopen([folder,'_conf_simulation_vertex']);
fid = fopen([folder,'\SimulationPath_testIRIS_vertex']);

tline = fgetl(fid);
vertexCell = cell(1,1);
vertexCell{1} = tline;
index = 0;
POI = 0;
indexPOI = 0;
POICell = cell(1,1);
while ischar(tline)
%     [aa,bb] = find(vertexTrajectory ==index);
%     if (isempty(aa))
%         index=index+1;
%         tline = fgetl(fid);
%         continue
%     end
        
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
edgedata = importdata([folder,'\SimulationPath_testIRIS_edge']);
%  = importdata([folder,'_conf_simulation_edge']);
collisionCheck = edgedata(:,4);

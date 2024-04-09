clc
clear

%% Include path
addpath InitFunction
addpath ServiceFunction
addpath Control
addpath Command
addpath PostProcess
addpath StepFunction
addpath Movie_function
%% load command infromation from IRIS planing
IRIS_build_folder = '\\wsl.localhost\Ubuntu\home\davidalpert11\Projects\IRIS-UU\build';
Result_path = 'Results\test\';
file_to_write = 'testIRIS';
OutputIRISFolderName = [IRIS_build_folder,'\',Result_path];

OutputSimulationPath = 'OutputSimulation';
Command = InitCommand(file_to_write,OutputIRISFolderName);

%% Run the simulation
mainUAVsimulation(Command);

%% Check POI and Collision

%path to the output result of iris (in the ubuntu machine)
%the file has to be reconise in ubuntu
load([OutputSimulationPath,'\UAVSimulationResults']);
Result_path_simulation_vertex_relative_to_build = [Result_path,'SimulationPath_',file_to_write];


[IRIS_build_folder,'\Results\test'];
Result_path_simulation_vertex = [IRIS_build_folder,'\',Result_path_simulation_vertex_relative_to_build];
fileName = fopen(Result_path_simulation_vertex,'w');
fprintf(fileName, '%f %f %f %f %f\n', PoseUpdateTheta');
% Closing
fclose(fileName)


seed =num2str(2);

CommandToPowershell = ['powershell cd ',IRIS_build_folder,'; wsl '];
system([CommandToPowershell,'./app/checkPOIandCollision_simulator ',strrep(Result_path_simulation_vertex_relative_to_build, '\', '/'),' ',seed])

%% Creat a video
obj_path = '\\wsl.localhost\Ubuntu\home\davidalpert11\Projects\IRIS-UU\data\bridge\bridge.obj';
output_record_path_simulation = 'OutputSimulation';
output_iris ='\\wsl.localhost\Ubuntu\home\davidalpert11\Projects\IRIS-UU\build\Results\test';
nameOfVideoFile = 'Movies\test_video';

Movie(nameOfVideoFile,obj_path,output_record_path_simulation,output_iris);


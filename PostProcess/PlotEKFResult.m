% function PlotEKFResult(RecordState,RecordNavState,RecordEKF,IMUParameters,t,SaveResults)
close all
SaveResults = 0;
plot_State=1;
t = [1:1:length(RecordState.X)]';
x_true =[RecordState.X;RecordState.X_dot;...
    RecordState.phi;RecordState.theta;RecordState.psi;...
    RecordState.bias_acc;RecordState.bias_gyro];
x_nav =[RecordNavState.X;RecordNavState.X_dot;...
    RecordNavState.phi;RecordNavState.theta;RecordNavState.psi;...
    RecordNavState.bias_acc;RecordNavState.bias_gyro];

% mis_eps = zeros(3,1);
% for i=1:1:Quad.counter-1
%     delta_Euler = [Quad.phi_plot(i)-Quad.phi_plot_meas(i);...
%     Quad.theta_plot(i)-Quad.theta_plot_meas(i);...
%     Quad.psi_plot(i)-Quad.psi_plot_meas(i)];
%     H_DELTA_EULER = [cos(Quad.psi_plot(i))/cos(Quad.theta_plot(i)),sin(Quad.psi_plot(i))/cos(Quad.theta_plot(i)),0;...
%                      -sin(Quad.psi_plot(i)),cos(Quad.psi_plot(i)),0;...
%                      tan(Quad.theta_plot(i))*cos(Quad.psi_plot(i)),tan(Quad.theta_plot(i))*sin(Quad.psi_plot(i)),1];
%     mis_eps(:,i) = (H_DELTA_EULER)\delta_Euler;
%
% end

%   Calculate real error
delta_x_true = [RecordNavState.X-RecordState.X;RecordNavState.X_dot-RecordState.X_dot;...
    RecordNavState.phi-RecordState.phi;RecordNavState.theta-RecordState.theta;RecordNavState.psi-RecordState.psi;...
    RecordNavState.bias_acc-IMUParameters.b_a;RecordNavState.bias_gyro-IMUParameters.b_g];


sigma_factor =3;
%   Calculate sleeves from covariance matrix
delta_x_upperBound = zeros(15,1) + sigma_factor*RecordEKF.P_diag_sqrt;
delta_x_lowerBound = zeros(15,1) - sigma_factor*RecordEKF.P_diag_sqrt;


x_upperBound = x_true + abs(delta_x_upperBound(1:15,:));
x_lowerBound = x_true - abs(delta_x_lowerBound(1:15,:));

%   Plot properties
legendLocation = 'best';
legendFontSize = 7;
path_for_results = 'Results';

%   Create directory for results if not exists
if exist(path_for_results, 'dir')
    warning('The directory "%s" already exists', path_for_results)
else
    mkdir(path_for_results)
end


if (plot_State)
    %%%%%%%%%%  Plot State results    %%%%%%%%%
    state_fig_name = 'State Results';
    state_fig_path = fullfile(path_for_results,state_fig_name);
    state_fig = figure('Name',state_fig_name,'units','normalized','outerposition',[0 0 1 1]);
    
    %   Plot position in geographic coordinates and velocity in NED coordinates
    titles = {'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz','\phi', '\theta', '\psi'};
    ylabels = {'X', 'Y', 'Z', 'Vx', 'Vy', 'Vz','\phi', '\theta', '\psi'};
    units = cell(1,6);
    units(1:3) = {'[m]','[m]','[m]'};
    units(4:6) = repmat({'[m/s]'}, 1, 3);
    units(7:9) = repmat({'[\circ]'},1,3);
    
    num_of_titles = length(titles);
    if (num_of_titles~=length(ylabels)) || (num_of_titles~=length(units))
        error('Number of titles do not match the number of other entries in figure "%s". Please validate dimension consistency.', state_fig_name);
    end
    
    for i = 1:num_of_titles
        subplot(3,3,i);
        hold on;grid on;box on;
        title([titles{i} ' Vs. Time']);
        xlabel('Time [second]');
        ylabel([ylabels{i} ' ' units{i}]);
        if (i<7)
            plot(t,(x_nav(i,:)),'LineWidth',2);
            plot(t,([x_upperBound(i,:); x_lowerBound(i,:)]),':r','LineWidth',2);
        else
            plot(t,rad2deg(x_nav(i,:)),'LineWidth',2);
            plot(t,rad2deg([x_upperBound(i,:); x_lowerBound(i,:)]),':r','LineWidth',2);
        end
        legend({ylabels{i},[ylabels{i} '\pm' num2str(sigma_factor) '\sigma']},'Location',legendLocation,'FontSize',legendFontSize);
    end
    
    %   Save results
    if (SaveResults)
        savefig(state_fig, state_fig_path, 'compact');  %   Compressed fig. Can only be opened in MATLAB 2014b or above.
        print(state_fig, state_fig_path, '-dpdf', '-bestfit');
    end
end
%%
%%%%%%%%%%%  Plot true error results    %%%%%%%%%
error_fig_name = 'Error State Results';
error_fig_path = fullfile(path_for_results,error_fig_name);
error_fig = figure('Name',error_fig_name,'units','normalized','outerposition',[0 0 1 1]);
legendLocation = 'best';

%   Plot position in geographic coordinates and velocity in NED coordinates
titles = {'\deltaX', '\deltaY ', '\deltaZ ', '\deltaV_x ', '\deltaV_y ', '\deltaV_z ','\delta\phi', '\delta\theta', '\delta\psi'};
ylabels =  {'\deltaX', '\deltaY ', '\deltaZ ', '\deltaV_x ', '\deltaV_y ', '\deltaV_z ','\delta\phi', '\delta\theta', '\delta\psi'};
units = cell(1,6);
units(1:3) = {'[m]','[m]','[m]'};
units(4:6) = repmat({'[m/s]'}, 1, 3);
units(7:9) = repmat({'[\circ]'},1,3);
num_of_titles = length(titles);
if (num_of_titles~=length(ylabels)) || (num_of_titles~=length(units))
    error('Number of titles do not match the number of other entries in figure "%s". Please validate dimension consistency.', state_fig_name);
end

for i = 1:num_of_titles
    subplot(3,3,i);
    hold on;grid on;box on;
    title([titles{i} ' Vs. Time']);
    xlabel('Time [second]');
    ylabel([ylabels{i} ' ' units{i}]);
    if (i==1) || (i==2)
        plot(t,(delta_x_true(i,:)),'LineWidth',2);
        plot(t,([delta_x_upperBound(i,:); delta_x_lowerBound(i,:)]),':r','LineWidth',2);
    else
        plot(t,delta_x_true(i,:),'LineWidth',2);
        plot(t,[delta_x_upperBound(i,:); delta_x_lowerBound(i,:)],':r','LineWidth',2);
    end
    legend({ylabels{i},[ylabels{i} '\pm' num2str(sigma_factor) '\sigma']},'Location',legendLocation,'FontSize',legendFontSize);
end
%   Save results
if (SaveResults)
    savefig(error_fig, error_fig_path, 'compact');  %   Compressed fig. Can only be opened in MATLAB 2014b or above.
    print(error_fig, error_fig_path, '-dpdf', '-bestfit');
end
%%
%%%%%%%%%%%  Plot bias results    %%%%%%%%%
bias_fig_name = 'Bias Results';
bias_fig_path = fullfile(path_for_results,bias_fig_name);
bias_fig = figure('Name',bias_fig_name,'units','normalized','outerposition',[0 0 1 1]);

%   Constants
milli_g2mpss = 9.806/1e3;           %   Conversion from [mili g] to [m/s^2]
degPerHr2radPerSec = (pi/180)/3600; %   Conversion from [deg/hr] to [rad/s]
%   Plot position in geographic coordinates and velocity in NED coordinates
titles = {'Accelerometer X','Accelerometer Y','Accelerometer Z', 'Gyro X', 'Gyro Y', 'Gyro Z'};
ylabels = {'Accelerometer X','Accelerometer Y','Accelerometer Z', 'Gyro X', 'Gyro Y', 'Gyro Z'};
units = cell(1,6);
units(1:3) = repmat({'[milli g]'}, 1, 3);
units(4:6) = repmat({'[\circ/h]'}, 1, 3);

num_of_titles = length(titles);
if (num_of_titles~=length(ylabels)) || (num_of_titles~=length(units))
    error('Number of titles do not match the number of other entries in figure "%s". Please validate dimension consistency.', state_fig_name);
end

for i = 1:num_of_titles
    subplot(2,3,i);
    hold on;grid on;box on;
    title([titles{i} ' Vs. Time']);
    xlabel('Time [second]');
    ylabel([ylabels{i} ' ' units{i}]);
    if (i<4)
        plot(t,(delta_x_true(i+9,:))./milli_g2mpss,'LineWidth',2);
        plot(t,([delta_x_upperBound(i+9,:); delta_x_lowerBound(i+9,:)])./milli_g2mpss,':r','LineWidth',2);
    else
        plot(t,delta_x_true(i+9,:)./degPerHr2radPerSec,'LineWidth',2);
        plot(t,[delta_x_upperBound(i+9,:); delta_x_lowerBound(i+9,:)]./degPerHr2radPerSec,':r','LineWidth',2);
    end
    legend({ylabels{i},[ylabels{i} '\pm' num2str(sigma_factor) '\sigma']},'Location',legendLocation,'FontSize',legendFontSize);
end

%   Save results
if (SaveResults)
    savefig(bias_fig, bias_fig_path, 'compact');  %   Compressed fig. Can only be opened in MATLAB 2014b or above.
    print(bias_fig, bias_fig_path, '-dpdf', '-bestfit');
end
% save([pwd,'\Results\RecordResult'],'RecordState','RecordNavState','RecordEKF','IMUParameters','t')
% end
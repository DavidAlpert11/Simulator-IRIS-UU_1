function Movie(nameOfVideoFile,obj_path,output_record_path_simulation,output_iris)


CreateVideoFromMatlab = 1;

init_plot;
define_quad_model;
    %

    Environment = GetEnvironmentMission(obj_path);

    % Define custom colormap for the bridge structure
    numColors = 256; % Number of colors in the colormap
    brownColor = [0.5 0.25 0]; % Brown color for the bridge structure
    grayColor = [0.5 0.5 0.5]; % Gray color for the bridge structure
    greenColor = [0 0.5 0]; % Green color for the bridge structure

    % Create custom colormap with smooth transition between colors
    customColorMap = zeros(numColors, 3);
    for i = 1:3
        customColorMap(:, i) = [linspace(brownColor(i), grayColor(i), numColors/2), ...
            linspace(grayColor(i), greenColor(i), numColors/2)].';
    end

    % Plot the bridge with realistic colors
    h1 = patch('vertices', [Environment.obj.v(:,1),Environment.obj.v(:,2),Environment.obj.v(:,3)], 'faces', Environment.obj.f.v, ...
        'FaceVertexCData', Environment.obj.v(:,3), 'FaceColor', 'interp', 'EdgeColor', 'none');

    % Apply custom colormap to the bridge structure
    colormap(customColorMap);

    % Adjust transparency and lighting
    shading interp
    lighting gouraud
    light('Position',[1 0 1]);
    material shiny
    alpha(.3)  % Adjust transparency

    %%
    %load
    load([output_record_path_simulation,'\UAVSimulationResults.mat'])

    vertex = readtable([output_iris,'\testIRIS_vertex']);
    all_poi_indexes  = extractUniquePOI(vertex);
    edge = importdata([output_iris,'\testIRIS_edge']);
    configuration = importdata([output_iris,'\testIRIS_conf']);


    % [aa,bb] = find(edge(:,4)>-2);

    %toy scenario
    aa = 1:1:length(edge);
    soucre = configuration(edge(aa,1)+1,1:3);
    target = configuration(edge(aa,2)+1,1:3);
    h3 =  plot3(soucre(:,1),soucre(:,2),soucre(:,3),'*','Color',[0, 0, 0, 0.7],'linewidth',0.5);
    plot3(target(:,1),target(:,2),target(:,3),'*','Color',[0, 0, 0, 0.7],'linewidth',0.5);

    %  text(configuration(:,1),configuration(:,2)+0.3,configuration(:,3),num2cell([0:1:length(configuration(:,1))-1]))

    %     plot3(target(:,1),target(:,2),target(:,3),'b*','linewidth',3);
    h_edges = plot3([soucre(:,1),target(:,1)]',[soucre(:,2),target(:,2)]',[soucre(:,3),target(:,3)]','--','Color',[0, 0, 0, 0.3]);

    [POIInspectedCell,collisionCheck] = GetInspectedPOIandCollision(output_iris);

    currentIndexesPOI = POIInspectedCell{1}+1;
    % h4 = plot3(Environment.TargetsBridge(:,1),Environment.TargetsBridge(:,2),Environment.TargetsBridge(:,3),'b*','LineWidth',3);
    currentIndexesPOICommand = Command.POIINDEX+1;
    POI_desire = num2str(length(unique(Command.POIINDEX)));

    h5 = plot3(Environment.TargetsBridge(currentIndexesPOICommand,1),Environment.TargetsBridge(currentIndexesPOICommand,2),Environment.TargetsBridge(currentIndexesPOICommand,3),'r*','LineWidth',3);

    % h4 = plot3(Environment.TargetsBridge(currentIndexesPOI,1),Environment.TargetsBridge(currentIndexesPOI,2),Environment.TargetsBridge(currentIndexesPOI,3),'r*','LineWidth',3);


    dt = 0.01;
    t=[1:1:length(RecordState.X(1,:))]*dt;
    time =t;




    %%
    % axis square;
    axis equal;

    plot_quad_model;
    h1 = animatedline('Color','b','LineWidth',2);
    h2 = animatedline('Color','b','LineWidth',2);
    h3 = animatedline('Color','k','LineWidth',2);
    % h4 = animatedline('Marker','o','Color','b','LineStyle','none');


    % addpoints(h4,Environment.TargetsBridge(:,1),Environment.TargetsBridge(:,2),Environment.TargetsBridge(:,3));
    % addpoints(h1,RecordState.X(1,:),RecordState.X(2,:),RecordState.X(3,:))
    % addpoints(h2,RecordNavState.X(1,:),RecordNavState.X(2,:),RecordNavState.X(3,:))
    % addpoints(h3,Command.Pose_des_GF(:,1),Command.Pose_des_GF(:,2),Command.Pose_des_GF(:,3))

    addpoints(h1,RecordState.X(1,1:2),RecordState.X(2,1:2),RecordState.X(3,1:2))
    % addpoints(h2,RecordNavState.X(1,1:2),RecordNavState.X(2,1:2),RecordNavState.X(3,1:2))
    addpoints(h3,Command.Pose_des_GF(1:2,1),Command.Pose_des_GF(1:2,2),Command.Pose_des_GF(1:2,3))

    % legend([h1,h2,h3],'True','Nav','Desire','location','best')
    % view([ -69.1836,  11.7112]);
    % view([  -55.3504,     15.9789]);
    % view([ -50.1836,  15.7112]);% view(3);



   
   azimuth_angle =   -18.4309%;-75;%75;102;-102;-75;
elevation_angle =20.3187;

    view([ azimuth_angle    elevation_angle]);

    % v = [-5 -2 5];ch, nr
    % [caz,cel] = view(v)
    % xlim([[RecordState.X(1,1)]-10,[RecordState.X(1,1)]+10])

    xlim([-120,120])
    indexInspectedPOI = 0;
    i=1;
    h6 = plot3([RecordState.X(1,i)-1000,RecordState.X(1,i)-1001],[RecordState.X(2,i),RecordState.X(2,i)+1],[RecordState.X(3,i),RecordState.X(3,i)+1],'x-','color',[0.9290, 0.6940, 0.1250],'LineWidth',14)
    h7 = plot3([RecordState.X(1,i)-1000,RecordState.X(1,i)-1001],[RecordState.X(2,i),RecordState.X(2,i)+1],[RecordState.X(3,i),RecordState.X(3,i)+1],'g*','LineWidth',3);

    % xlim([[RecordState.X(1,1)]-50,[RecordState.X(1,1)]+50])

    NumberOfCollision = 0;

    % if (isField(F))
    clear F
    % end
    indexF = 0;
    lastIndex= 1;
%%
    for i=1:1:length(time)
        if (i+1<length(time))
            addpoints(h1,RecordState.X(1,i:i+1),RecordState.X(2,i:i+1),RecordState.X(3,i:i+1))
            % addpoints(h2,RecordNavState.X(1,i:i+1),RecordNavState.X(2,i:i+1),RecordNavState.X(3,i:i+1))
        end
        % if (length(Command.Pose_des_GF) >= indexInspectedPOI+2)
        % end
        %     Plot the Quadrotor's Position
        if(mod(i,5)==0)
            plot_quad
            % xlim([max([RecordState.X(1,i)-70,30]),min([RecordState.X(1,i)+70,130])])
            % Add or update text outside the figure
            if exist('hTimeText', 'var')
                delete(hTimeText); % Delete existing text object
            end
            hTimeText = annotation('textbox', [0.5, 0.2, 0.1, 0.1], ...
                'String', ['Time: ', num2str(int32(time(i))),' [sec]'], ...
                'Color', 'red', ...
                'FontSize', 24, ...
                'EdgeColor', 'none'); % Remove the frame around the text box
            if (abs(Command.PoseSimulation(indexInspectedPOI+1,7)-i)<6)


                indexInspectedPOI = indexInspectedPOI+1;
                if (length(Command.Pose_des_GF) > indexInspectedPOI+1)
                    addpoints(h3,Command.Pose_des_GF(indexInspectedPOI:indexInspectedPOI+1,1),Command.Pose_des_GF(indexInspectedPOI:indexInspectedPOI+1,2),Command.Pose_des_GF(indexInspectedPOI:indexInspectedPOI+1,3))
                end
                if (indexInspectedPOI<=length(collisionCheck))
                    if (~collisionCheck(indexInspectedPOI))
                        NumberOfCollision = NumberOfCollision+1;
                        plot3([RecordState.X(1,lastIndex),RecordState.X(1,i)],[RecordState.X(2,lastIndex),RecordState.X(2,i)],[RecordState.X(3,lastIndex),RecordState.X(3,i)],'x-','color',[0.9290, 0.6940, 0.1250],'LineWidth',14)
                    end
                end
                lastIndex = i;
                currentIndexesPOI = POIInspectedCell{indexInspectedPOI}+1;
                %             currentIndexesPOICommand = Command.POICellCommand{indexInspectedPOI}+1;
                %             plot3(Environment.TargetsBridge(currentIndexesPOICommand,1),Environment.TargetsBridge(currentIndexesPOICommand,2),Environment.TargetsBridge(currentIndexesPOICommand,3),'r*','LineWidth',3);
                if (indexInspectedPOI<2)
                    totalPOI = currentIndexesPOI;

                    currentIndexesPOICommand = Command.POIINDEX+1;
                    currentPlanedPOI= (intersect(unique(currentIndexesPOI),unique(currentIndexesPOICommand)));
                    totalPlanedPOI = currentPlanedPOI;
                    % h5 = plot3(Environment.TargetsBridge(currentIndexesPOICommand,1),Environment.TargetsBridge(currentIndexesPOICommand,2),Environment.TargetsBridge(currentIndexesPOICommand,3),'r*','LineWidth',3);
                    h5 = plot3(Environment.TargetsBridge(all_poi_indexes+1,1),Environment.TargetsBridge(all_poi_indexes+1,2),Environment.TargetsBridge(all_poi_indexes+1,3),'r*','LineWidth',3);

                    
                    
                    h4 = plot3(Environment.TargetsBridge(currentIndexesPOI,1),Environment.TargetsBridge(currentIndexesPOI,2),Environment.TargetsBridge(currentIndexesPOI,3),'g*','LineWidth',3);
                    plot3(Environment.TargetsBridge(currentPlanedPOI,1),Environment.TargetsBridge(currentPlanedPOI,2),Environment.TargetsBridge(currentPlanedPOI,3),'g*','LineWidth',3);

                else
                    totalPOI = [totalPOI,currentIndexesPOI];
                    currentPlanedPOI= (intersect(unique(currentIndexesPOI),unique(currentIndexesPOICommand)));
                    totalPlanedPOI = [totalPlanedPOI,currentPlanedPOI];
                    plot3(Environment.TargetsBridge(currentIndexesPOI,1),Environment.TargetsBridge(currentIndexesPOI,2),Environment.TargetsBridge(currentIndexesPOI,3),'g*','LineWidth',3);
                    plot3(Environment.TargetsBridge(currentPlanedPOI,1),Environment.TargetsBridge(currentPlanedPOI,2),Environment.TargetsBridge(currentPlanedPOI,3),'g*','LineWidth',3);

                end

                %             legend([h1,h2,h3,h4,h5,h6],'True','Navigation','Desire',['Inspected POIs simulation = ',num2str(0),'    '],['Inspected POIs desired = ',num2str(length(unique(Command.POIINDEX))),'    '],['Number of collisions = ',num2str(NumberOfCollision),'    '],'location','northoutside','Fontsize',20)
                dummyh = line(nan, nan, 'Linestyle', 'none', 'Marker', 'none', 'Color', 'none');

                %             legend([h1,h2,h3,h4,h7,dummyh,h5,h6],'True','Navigation','Desire',['Additional Inspected POIs simulation = ',num2str(length(unique(setdiff(totalPOI,currentIndexesPOICommand)))),'    ']...
                %                 ,['Planned Inspected POIs simulation = ',num2str(length(unique(totalPlanedPOI))),'    ']...
                %                 ,['Total Inspected POIs simulation = ',num2str(length(unique(totalPOI)))]...
                %                 ,['Inspected POIs desired = ',num2str(length(unique(Command.POIINDEX))),'    ']...
                %                 ,['Number of collisions = ',num2str(NumberOfCollision),'    '],'location','bestoutside','Fontsize',20)

                %      legend([h1,h2,h3,h7,h5,h6]...
                %                 ,'True','Navigation','Desire'...
                %                 ,['Inspected POIs simulation = ',num2str(length(unique(totalPOI)))]...
                %                 ,['Inspected POIs desired = ',POI_desire,'    ']...
                %                 ,['Number of collisions = ',num2str(NumberOfCollision),'    '],'location','bestoutside','Fontsize',20)

                % ,['Additinal POIs = ',num2str(length(unique(setdiff(totalPOI,currentIndexesPOICommand))))]...
                % ,['All POI = ', num2str(length(all_poi_indexes)) ]...

                % legend([h3,h1,h5,h7,h6,h_edges(1)]...
                %     ,'Command path','Execution path'...
                %     ,['Command POIs = ',POI_desire,'    ']...
                %     ,['Execution POIs  = ',num2str(length(unique(totalPOI)))]...
                %     ,['Number of collisions = ',num2str(NumberOfCollision),'    ']...
                %     ,'Roadmap'...
                %     ,'location','southoutside','Fontsize',20,'Orientation', 'vertical','NumColumns',3);
                        legend([h3,h1,h5,h7,h6,h_edges(1)]...
                    ,'Command path','Execution path'...
                    ,['Command POIs = ',num2str(length(all_poi_indexes)),'    ']...
                    ,['Execution POIs  = ',num2str(length(unique(totalPOI))),' (',num2str(int32(100*length(unique(totalPOI))/length(all_poi_indexes))),'%)']...
                    ,['Number of collisions = ',num2str(NumberOfCollision),'    ']...
                    ,'Roadmap'...
                    ,'location','southoutside','Fontsize',20,'Orientation', 'vertical','NumColumns',3);

            end

            %         campos([Quad.X_plot(i)+2,Quad.Y_plot(i)+2,Quad.Z_plot(i)+2])
            %         camtarget([Quad.X_plot(i),Quad.Y_plot(i),Quad.Z_plot(i)])
            %         camroll(0);
            %         addpoints(h1,Quad.X_plot(i),Quad.Y_plot(i),-Quad.Z_plot(i))
            %         addpoints(h2,Quad.X_plot(i),Quad.Y_plot(i),-Quad.Z_plot(i))
            if i>=1

% Define the predefined set of azimuth angles
azimuth_angles = [75, 102, -102, -75];

% Bridge dimensions
x_bridge = [-100, 100];
y_bridge = [-11, 2];
z_bridge = [-30, 0];

% Calculate the center of the bridge
center_bridge = [mean(x_bridge), mean(y_bridge), mean(z_bridge)];

% Define threshold zones for x+ (right side of the bridge), x- (left side of the bridge), y+ (upper side of the bridge), and y- (lower side of the bridge)
x_threshold = 75; % Adjust as needed
y_threshold = 6;  % Adjust as needed

% Drone position (example)
drone_position = [RecordState.X(1,i),RecordState.X(2,i),RecordState.X(3,i)];

% Calculate the drone's position relative to the center of the bridge
delta_x = drone_position(1) - center_bridge(1);
delta_y = drone_position(2) - center_bridge(2);

% Initialize the selected azimuth angle
selected_azimuth_angle = [];

% Check the drone's position relative to the threshold zones
if abs(delta_x) <= x_threshold
    % Drone is within the x_threshold zone
    if delta_y >= 0
        % Drone is on the upper side of the bridge
        selected_azimuth_angle = azimuth_angles(1); % Choose the first azimuth angle
    else
        % Drone is on the lower side of the bridge
        selected_azimuth_angle = azimuth_angles(4); % Choose the last azimuth angle
    end
elseif abs(delta_y) <= y_threshold
    % Drone is within the y_threshold zone
    if delta_x >= 0
        % Drone is on the right side of the bridge
        selected_azimuth_angle = azimuth_angles(2); % Choose the second azimuth angle
    else
        % Drone is on the left side of the bridge
        selected_azimuth_angle = azimuth_angles(3); % Choose the third azimuth angle
    end
else
    % Drone is outside both threshold zones
    % Choose the closest predefined azimuth angle
    [~, index] = min(abs(delta_x));
    selected_azimuth_angle = azimuth_angles(index);
end

lambda = 0.2;
azimuth_angle = azimuth_angle*(1-lambda) + selected_azimuth_angle*lambda;
    % view([ azimuth_angle    elevation_angle]);

            % v = [RecordState.X(1,i),RecordState.X(2,i),RecordState.X(3,i)]-[RecordState.X(1,i-1),RecordState.X(2,i-1),RecordState.X(3,i-1)];
            %         [caz,cel] = view(v);
            end
            %         LimitView = 30;
            %
            %         if (caz <-LimitView)
            %             caz = -LimitView;
            %         end
            %         if ( caz >LimitView)
            %             caz = LimitView;
            %         end
            %           if (cel <-LimitView)
            %             cel = -LimitView;
            %         end
            %         if ( cel >LimitView)
            %             cel = LimitView;
            %         end
            %
            %
            %         view([caz,cel]);
            if (CreateVideoFromMatlab && (mod(i,50)==0))
                indexF = indexF+1;
                F(indexF) = getframe(gcf) ;
                %          cdata = print('-RGBImage','-r120');
                %         F(indexF) = im2frame(cdata);
            end
            % campos([RecordState.X(1:3,i)']);
            % view([RecordState.psi(i),RecordState.theta(i)])
            drawnow limitrate
            %         legend('off')
            % drawnow
        end
    end
    %%
    
    if (CreateVideoFromMatlab)
        save([nameOfVideoFile],"F");
        % create the video writer with 1 fps
        writerObj = VideoWriter([nameOfVideoFile,'.avi']);
        writerObj.FrameRate = 60;
        % set the seconds per image
        % open the video writer
        open(writerObj);
        % write the frames to the video
        for i=1:length(F)
            % convert the image to a frame
            frame = F(i) ;
            writeVideo(writerObj, frame);
        end
        % close the writer object
        close(writerObj);
    end
end
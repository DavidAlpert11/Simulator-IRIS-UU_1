function Control = InitControl

% Control Inputs
Control.U1 = 0;       % Total thrust (N)
Control.U2 = 0;       % Torque about X axis BF (N-m)
Control.U3 = 0;       % Torque about Y axis BF (N-m)
Control.U4 = 0;       % Torque about Z axis BF (N-m)

factorLim = 1;
% Control Limits (update values)
Control.U1_max = factorLim*43.5;   % Control.KT*4*Control.max_motor_speed^2
Control.U1_min = factorLim*0;      % 
Control.U2_max = factorLim*6.25;  % Control.KT*Control.l*Control.max_motor_speed^2
Control.U2_min = factorLim*-6.25; % Control.KT*Control.l*Control.max_motor_speed^2
Control.U3_max = factorLim*6.25;  % Control.KT*Control.l*Control.max_motor_speed^2
Control.U3_min = factorLim*-6.25; % Control.KT*Control.l*Control.max_motor_speed^2
Control.U4_max = factorLim*2.25; % Control.Kd*2*Control.max_motor_speed^2
Control.U4_min = factorLim*-2.25;% Control.Kd*2*Control.max_motor_speed^2

factorLim = 1;
% PID parameters
Control.X_KP = .35;          % KP value in X position control
Control.X_KI = .25;            % KI value in X position control
Control.X_KD = -.35;         % KD value in X position control
Control.X_KI_lim = factorLim*.25;         % Error to start calculating integral term

Control.Y_KP = .35;          % KP value in Y position control
Control.Y_KI = .25;            % KI value in Y position control
Control.Y_KD = -.35;         % KD value in Y position control
Control.Y_KI_lim = factorLim*.25;         % Error to start calculating integral term

Control.Z_KP = 10/1.7;    % KP value in altitude control
Control.Z_KI = 0*3;    % KI value in altitude control
Control.Z_KD = -10/1.980;  % KD value in altitude control
Control.Z_KI_lim = factorLim*.25;         % Error to start calculating integral term

Control.phi_KP = 4.5;      % KP value in roll control 2
Control.phi_KI = 0;       % KI value in roll control   1        
Control.phi_KD = 0;     % KD value in roll control  -.5
Control.phi_max = pi/4;   % Maximum roll angle commanded
Control.phi_KI_lim = factorLim*2*(2*pi/360);  % Error to start calculating integral 

Control.theta_KP = 4.5;    % KP value in pitch control 2
Control.theta_KI = 0;     % KI value in pitch control 1
Control.theta_KD = 0;   % KD value in pitch control -.5
Control.theta_max = pi/4; % Maximum pitch angle commanded
Control.theta_KI_lim = factorLim*2*(2*pi/360);  % Error to start calculating integral 

Control.psi_KP = 10;     % KP value in yaw control
Control.psi_KI = 0;     % KI value in yaw control .75
Control.psi_KD = 0;     % KD value in yaw control -.5
Control.psi_KI_lim = factorLim*8*(2*pi/360);  % Error to start calculating integral 

Control.p_KP = 2.7;    % KP value in pitch control 2
Control.p_KI = 1;     % KI value in pitch control
Control.p_KD = -.01;   % KD value in pitch control -.5
Control.p_max = 50*(2*pi/360); % Maximum pitch angle commanded
Control.p_KI_lim = factorLim*10*(2*pi/360);  % Error to start calculating integral 

Control.q_KP = 2.7;    % KP value in pitch control
Control.q_KI = 1;     % KI value in pitch control
Control.q_KD = -.01;   % KD value in pitch control -.5
Control.q_max = 50*(2*pi/360); % Maximum pitch angle commanded
Control.q_KI_lim = factorLim*10*(2*pi/360);  % Error to start calculating integral 

Control.LimitCommand = 1.5;%[m]

Control.r_KP = 2.7;    % KP value in pitch control
Control.r_KI = 1;     % KI value in pitch control
Control.r_KD = -.01;   % KD value in pitch control
Control.r_max = 50*(2*pi/360); % Maximum pitch angle commanded
Control.r_KI_lim = factorLim*10*(2*pi/360);  % Error to start calculating integral 

Control.init=0;
Control.phi_des = 0;
Control.theta_des = 0;

Control.max_motor_speed = 925; % motors upper limit (rad/s)
Control.min_motor_speed = 0; %-1*((400)^2); % motors lower limit (can't spin in reverse)

Control.Obar = 0;     % sum of motor speeds (O1-O2+O3-O4, N-m) 
Control.O1 = 0;       % Front motor speed (raidans/s)
Control.O2 = 0;       % Right motor speed (raidans/s)
Control.O3 = 0;       % Rear motor speed (raidans/s)
Control.O4 = 0;       % Left motor speed (raidans/s)
end
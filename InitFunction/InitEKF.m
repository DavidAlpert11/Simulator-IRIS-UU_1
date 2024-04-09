function EKF = InitEKF(NavState,IMUParameters,ScenarioParameter)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
sigma_delta_p0 = NavState.SigmaPosInit*ones(3,1);
sigma_delta_v0 = NavState.SigmaVelInit*ones(3,1);
sigma_eulerAngle0 = NavState.SigmaEulerInit*ones(3,1);
sigma_b_a0 = IMUParameters.b_a;
sigma_b_g0 = IMUParameters.b_g;
sigma_vec_ES0 = [sigma_delta_p0; sigma_delta_v0;   sigma_eulerAngle0; sigma_b_a0; sigma_b_g0];    %   Vector of all the uncertainties in the intial error state
EKF.delta_x = zeros(15,1) + sigma_vec_ES0.*rand(length(sigma_vec_ES0),1);    %   Add noise to the initial error state
EKF.P = diag(sigma_vec_ES0.^2);
%   Measurement noise uncertainty [m^2]
EKF.R = diag([sigma_delta_p0',sigma_delta_v0'].^2);

% EKF.R = diag(sigma_delta_p0'.^2);

%   Bias Gauss-Markov process matrices (not including noise, which is accounted for in the Q matrix)
Gauss_markov = 0;
if (Gauss_markov)
    %   Sensors bias parameters (Gauss-Markov process paramters)
    tau_a = 100;                          %   Accelerometer bias process charectaristic time [sec]
    tau_g = 100;                         %   Gyro bias process charectaristic time [sec]
    F_a = -1/tau_a*eye(3);
    F_g = -1/tau_g*eye(3);
    sigma_agm = IMUParameters.w_a/tau_a;        %   Accelerometer bias process noise std [m/s^2/s]
    sigma_ggm = IMUParameters.w_g/tau_g;  %   Gyro bias process noise std [rad/s/s]
else
    sigma_ggm = 0*ones(3,1);
    sigma_agm = 0*ones(3,1);
    F_a = 0*eye(3);
    F_g = 0*eye(3);
end
%   Process noise (variances along the diagonal)
EKF.Q = ScenarioParameter.dt*diag([IMUParameters.w_a*ones(3,1);  IMUParameters.w_g*ones(3,1);    sigma_agm;  sigma_ggm].^2);
% EKF.Q = diag([IMUParameters.w_a*ones(3,1);  IMUParameters.w_g*ones(3,1);    sigma_agm;  sigma_ggm].^2);

EKF.GM = [F_a,zeros(3);...
    zeros(3),F_g];    %   Total Gauss-Markov matrix

EKF.H =  [eye(6),zeros(6,length(EKF.delta_x)-6)];          %   Measurement matrix
% EKF.H =  [eye(3),zeros(3,length(EKF.delta_x)-3)];          %   Measurement matrix
end


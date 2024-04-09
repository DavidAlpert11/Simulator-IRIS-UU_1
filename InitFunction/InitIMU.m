function IMUParameters = InitIMU(ScenarioParameter)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
milli_g2mpss = 9.806/1e3;           %   Conversion from [mili g] to [m/s^2]
degPerHr2radPerSec = (pi/180)/3600; %   Conversion from [deg/hr] to [rad/s]
dt = ScenarioParameter.dt;

IMUParameters.b_a =10*milli_g2mpss*ones(3,1);
% IMUParameters.b_a(3) = 0;%todo simple test only
IMUParameters.w_a = 1*milli_g2mpss;%/sqrt(ScenarioParameter.dt);
IMUParameters.M_a = zeros(3,3);
IMUParameters.S_a = zeros(3,3);

IMUParameters.b_g = 10*degPerHr2radPerSec *ones(3,1);
IMUParameters.w_g = 1*degPerHr2radPerSec;%/sqrt(ScenarioParameter.dt);
IMUParameters.M_g = zeros(3,3);
IMUParameters.S_g = zeros(3,3);
IMUParameters.G = zeros(3,3);     %   Sensitivity of gyro to linear acceleration


if (ScenarioParameter.IdealIMU)
    IMUParameters.b_a = 0*milli_g2mpss*ones(3,1);
    IMUParameters.w_a = 0*milli_g2mpss;%/sqrt(ScenarioParameter.dt);
    IMUParameters.b_g = 0*degPerHr2radPerSec *ones(3,1);
    IMUParameters.w_g = 0*degPerHr2radPerSec;%/sqrt(ScenarioParameter.dt);
end
IMUParameters.tau_a = 100;
IMUParameters.tau_g = 100;
IMUParameters.sigma_acc_gm = IMUParameters.w_a/1000;
IMUParameters.sigma_gyro_gm = IMUParameters.w_g/1000;
IMUParameters.current_bias_acc =IMUParameters.b_a;
IMUParameters.current_bias_gyro =IMUParameters.b_g;
end


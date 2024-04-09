function SensorMeas = GetSensorMeas(DynamicStateParameter,IMUParameters)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here


% b_a_dot = -IMUParameters.current_bias_acc./IMUParameters.tau_a + IMUParameters.sigma_acc_gm.*randn(3,1);
% b_g_dot = -IMUParameters.current_bias_gyro./IMUParameters.tau_g + IMUParameters.sigma_gyro_gm.*randn(3,1);
IMUParameters.current_bias_acc =IMUParameters.b_a+IMUParameters.w_a*randn(3,1)+(IMUParameters.M_a+IMUParameters.S_a)*DynamicStateParameter.f_b;
IMUParameters.current_bias_gyro =IMUParameters.b_g+IMUParameters.w_g*randn(3,1)+(IMUParameters.M_g+IMUParameters.S_g)*DynamicStateParameter.w_b+IMUParameters.G*DynamicStateParameter.f_b;

SensorMeas.f = DynamicStateParameter.f_b+IMUParameters.current_bias_acc;
SensorMeas.w = DynamicStateParameter.w_b+IMUParameters.current_bias_gyro;

end


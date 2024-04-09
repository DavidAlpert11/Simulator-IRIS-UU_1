function NavStateNew = UpdateNavStateFromEKF(delta_x,NavState,C_B2I_estimate)

%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here

NavStateNew = NavState;
NavStateNew.X = NavState.X - delta_x(1:3);
NavStateNew.X_dot = NavState.X_dot - delta_x(4:6);

NavStateNew.C_B2I = (eye(3)-skew([delta_x(7),delta_x(8),delta_x(9)]))*C_B2I_estimate;

[ EulerAngles ] = dcm2euler( NavStateNew.C_B2I) ;
NavStateNew.phi = EulerAngles(1);
NavStateNew.theta = EulerAngles(2);
NavStateNew.psi = EulerAngles(3);
NavStateNew.bias_acc = NavState.bias_acc+delta_x(10:12);
NavStateNew.bias_gyro = NavState.bias_gyro+delta_x(13:15);
end


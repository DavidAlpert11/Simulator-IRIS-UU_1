function Control = UpdateControl(Control,NavState,X_des_GF,psi_des,DynamicNavStateParameter,Quad,g,dt)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Control = XYCommand2PhiTheta(Control,NavState,X_des_GF,dt);
Control = ZandEuler2PQRU1(Control,NavState,X_des_GF,psi_des,DynamicNavStateParameter.w_b,Quad.m,g,dt);
Control = PQR2U_234(Control,Quad,DynamicNavStateParameter.w_b,dt);

%     Calculate Desired Motor Speeds
Control = quad_motor_speed(Control,Quad);
end


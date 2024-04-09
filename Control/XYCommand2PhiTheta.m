% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A high level controller outputs desired roll and
% pitch angles based on errors between the Global and desired X and Y
% positions. A lower level controller takes those inputs and controls the
% error between the deisred and actual Euler angles. After the control
% inputs are calculated, the desired motor speeds are calculated. See
% www.wilselby.com for derivations of these equations.

function Control = XYCommand2PhiTheta(Control,NavState,X_des_GF,dt)

persistent x_error_sum;
persistent y_error_sum;

% initialize persistent variables at beginning of simulation
if Control.init==0
    x_error_sum = 0;
    y_error_sum = 0;
end

%% High Level Position Controller

C_I2B = NavState.C_B2I';

X_des_BF = C_I2B*X_des_GF;
% [Quad.X_des,Quad.Y_des,Quad.Z_des] = C_I2B*X_des_GF;

% Rotate Current Position from GF to BF
X_BF = C_I2B*NavState.X;
% [Quad.X_BF,Quad.Y_BF,Quad.Z_BF] = rotateGFtoBF(x,y,z,Quad.C_I2B_estimate);

% Rotate Current Velocity from GF to BF
X_BF_dot = C_I2B*NavState.X_dot;
% [Quad.X_BF_dot,Quad.Y_BF_dot,Quad.Z_BF_dot] = rotateGFtoBF(X_dot,Y_dot,Z_dot,Quad.C_I2B_estimate);

% X Position PID controller
x_error =X_des_BF(1)-X_BF(1);
% x_error = Quad.X_des - Quad.X_BF;

if abs(x_error_sum)>Control.LimitCommand
    x_error_sum = 0;
end
if(abs(x_error) < Control.X_KI_lim)
    x_error_sum = x_error_sum + x_error;
end

cp = Control.X_KP*x_error;    %Proportional term
ci = Control.X_KI*dt*x_error_sum;
ci = min(Control.theta_max, max(-Control.theta_max, ci));    %Saturate ci
cd = Control.X_KD*X_BF_dot(1);                     %Derivative term
Control.theta_des =  - (cp + ci + cd);   %Theta and X inversely related
Control.theta_des = min(Control.theta_max, max(-Control.theta_max, Control.theta_des));


% Y Position PID controller
y_error =X_des_BF(2)-X_BF(2);
% y_error = Quad.Y_des - Quad.Y_BF;

if abs(y_error_sum)>Control.LimitCommand
    y_error_sum = 0;
end
if(abs(y_error) < Control.Y_KI_lim)
    y_error_sum = y_error_sum + y_error;
    
end
cp = Control.Y_KP*y_error;    %Proportional term
ci = Control.Y_KI*dt*y_error_sum;
ci = min(Control.phi_max, max(-Control.phi_max, ci));    %Saturate ci
cd = Control.Y_KD*X_BF_dot(2);                      %Derivative term
Control.phi_des = cp + ci + cd;
Control.phi_des = min(Control.phi_max, max(-Control.phi_max, Control.phi_des));

end

















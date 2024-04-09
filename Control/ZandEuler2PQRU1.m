% Wil Selby
% Washington, DC
% May 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. A lower level controller takes those inputs and
% controls the error between the deisred and actual Euler angles.

function Control = ZandEuler2PQRU1(Control,NavState,X_des_GF,psi_des,pqr,m,g,dt)

persistent z_error_sum;
persistent phi_error_sum;
persistent theta_error_sum;
persistent psi_error_sum;


% initialize persistent variables at beginning of simulation
if Control.init==0
    z_error_sum = 0;
    phi_error_sum = 0;
    theta_error_sum = 0;
    psi_error_sum = 0;
end

phi = NavState.phi;
theta = NavState.theta;
psi = NavState.psi;
p = pqr(1);
q = pqr(2);
r = pqr(3);
Z = NavState.X(3);
Z_dot = NavState.X_dot(3);
%% Z Position PID Controller/Altitude Controller
z_error= X_des_GF(3)-Z;
% if (abs(z_error)>Control.LimitCommand)
%     z_error = sign(z_error)*Control.LimitCommand;
% end
if abs(z_error_sum)>Control.LimitCommand
    z_error_sum = 0;
end
if(abs(z_error) < Control.Z_KI_lim)
    z_error_sum = z_error_sum + z_error;
    
end
cp = Control.Z_KP*z_error;         %Proportional term
ci = Control.Z_KI*dt*z_error_sum; %Integral term
ci = min(Control.U1_max, max(Control.U1_min, ci));    %Saturate ci
cd = Control.Z_KD*Z_dot;                  %Derivative term
Control.U1 = -(cp + ci + cd)/(cos(theta)*cos(phi)) + (m * g)/(cos(theta)*cos(phi));   %Negative since Thurst and Z inversely related
Control.U1 = min(Control.U1_max, max(Control.U1_min, Control.U1));


%% Attitude Controller

% Roll PID Controller
phi_error = Control.phi_des - phi;
% if abs(phi_error_sum)>(Control.phi_KI_lim*4)
%         phi_error_sum = 0;
%     end
if(abs(phi_error) < Control.phi_KI_lim)
    phi_error_sum = phi_error_sum + phi_error;
end
cp = Control.phi_KP*phi_error;
ci = Control.phi_KI*dt*phi_error_sum;
ci = min(Control.p_max, max(-Control.p_max, ci));
cd = Control.phi_KD*p;
Control.p_des = cp + ci + cd;
Control.p_des = min(Control.p_max, max(-Control.p_max, Control.p_des));

% Pitch PID Controller
theta_error = Control.theta_des - theta;
% if abs(theta_error_sum)>(Control.theta_KI_lim*4)
%         theta_error_sum = 0;
%     end
if(abs(theta_error) < Control.theta_KI_lim)
    theta_error_sum = theta_error_sum + theta_error;
end
cp = Control.theta_KP*theta_error;
ci = Control.theta_KI*dt*theta_error_sum;
ci = min(Control.q_max, max(-Control.q_max, ci));
cd = Control.theta_KD*q;
Control.q_des = cp + ci + cd;
Control.q_des = min(Control.q_max, max(-Control.q_max, Control.q_des));


% Yaw PID Controller
psi_error = psi_des - psi;
% if abs(psi_error_sum)>(Control.psi_KI_lim*4)
%         psi_error_sum = 0;
%     end
if(abs(psi_error) < Control.psi_KI_lim)
    psi_error_sum = psi_error_sum + psi_error;
end
cp = Control.psi_KP*psi_error;
ci = Control.psi_KI*dt*psi_error_sum;
ci = min(Control.r_max, max(-Control.r_max, ci));
cd = Control.psi_KD*r;
Control.r_des = cp + ci + cd;
Control.r_des = min(Control.r_max, max(-Control.r_max, Control.r_des));

end

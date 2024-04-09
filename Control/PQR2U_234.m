% Wil Selby
% Washington, DC
% Sep 30, 2015

% This function implements a Proportional Integral Derivative Controller
% (PID) for the quadrotor. This is the lowest level controller. It recieved
% desired angular roll rates from the attitude controller. The outputs are 
% then sent directly to the motors. 

function Control = PQR2U_234(Control,Quad,pqr,dt)

persistent p_error_sum;
persistent q_error_sum;
persistent r_error_sum;

% initialize persistent variables at beginning of simulation
if Control.init==0
    p_error_sum = 0;
    q_error_sum = 0;
    r_error_sum = 0;
end

p = pqr(1);
q = pqr(2);
r = pqr(3);
pqr_dot = [(q*r*(Quad.Jy - Quad.Jz) - Quad.Jp*p*Control.Obar + Quad.l*Control.U2)/Quad.Jx;
(p*r*(Quad.Jz - Quad.Jx) + Quad.Jp*q*Control.Obar + Quad.l*Control.U3)/Quad.Jy;
(p*q*(Quad.Jx - Quad.Jy) + Control.U4)/Quad.Jz];
p_dot = pqr_dot(1);
q_dot = pqr_dot(2);
r_dot = pqr_dot(3);

%% Angular Rate Controller

% Roll PID Controller
p_error = Control.p_des - p;
% if abs(p_error_sum)>(Control.p_KI_lim*4)
%         p_error_sum = 0;
%     end
if(abs(p_error) < Control.p_KI_lim)
    p_error_sum = p_error_sum + p_error;
end
cp = Control.p_KP*p_error;
ci = Control.p_KI*dt*p_error_sum;
ci = min(Control.U2_max, max(Control.U2_min, ci));
cd = Control.p_KD*p_dot;
Control.U2 = cp + ci + cd;
Control.U2 = min(Control.U2_max, max(Control.U2_min, Control.U2));

% Pitch PID Controller
q_error = Control.q_des - q;
% if abs(q_error_sum)>(Control.q_KI_lim*4)
%         q_error_sum = 0;
%     end
if(abs(q_error) < Control.q_KI_lim)
    q_error_sum = q_error_sum + q_error;
end
cp = Control.q_KP*q_error;
ci = Control.q_KI*dt*q_error_sum;
ci = min(Control.U3_max, max(Control.U3_min, ci));
cd = Control.q_KD*q_dot;
Control.U3 = cp + ci + cd;
Control.U3 = min(Control.U3_max, max(Control.U3_min, Control.U3));

% Yaw PID Controller
r_error = Control.r_des - r;
% if abs(r_error_sum)>(Control.r_KI_lim*4)
%         r_error_sum = 0;
%     end
if(abs(r_error) < Control.r_KI_lim)
    r_error_sum = r_error_sum + r_error;
end
cp = Control.r_KP*r_error;
ci = Control.r_KI*dt*r_error_sum;
ci = min(Control.U4_max, max(Control.U4_min, ci));
cd = Control.r_KD*r_dot;
Control.U4 = cp + ci + cd;
Control.U4 = min(Control.U4_max, max(Control.U4_min, Control.U4));

end

















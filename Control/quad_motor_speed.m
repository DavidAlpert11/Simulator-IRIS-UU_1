% Wil Selby
% Washington, DC
% May 30, 2015

% This function converts the desired force and moment control inputs into
% the desired speed of the motors. These speeds are then limited by the
% physical properties of our motor. The conventional control commands are
% then re-computed with the limited motor speeds for input into the control
% system. Note that these motor speeds would be the signals sent to the
% ESCs on the actual quadrotor.


function Control = quad_motor_speed(Control,Quad)

% Calculate motor speeds (rad/s)^2
w1 = Control.U1/(4*Quad.KT) + Control.U3/(2*Quad.KT*Quad.l) + Control.U4/(4*Quad.Kd);
w2 = Control.U1/(4*Quad.KT) - Control.U2/(2*Quad.KT*Quad.l) - Control.U4/(4*Quad.Kd);
w3 = Control.U1/(4*Quad.KT) - Control.U3/(2*Quad.KT*Quad.l) + Control.U4/(4*Quad.Kd);
w4 = Control.U1/(4*Quad.KT) + Control.U2/(2*Quad.KT*Quad.l) - Control.U4/(4*Quad.Kd);

% Apply realistic motor speed limits
if w1 > Control.max_motor_speed^2
    w1 = Control.max_motor_speed^2;
end
if w1 < Control.min_motor_speed^2
    w1 = Control.min_motor_speed^2;
end

if w2 > Control.max_motor_speed^2
    w2 = Control.max_motor_speed^2;
end
if w2 < Control.min_motor_speed^2
    w2 = Control.min_motor_speed^2;
end

if w3 > Control.max_motor_speed^2
    w3 = Control.max_motor_speed^2;
end
if w3 < Control.min_motor_speed^2
    w3 = Control.min_motor_speed^2;
end

if w4 > Control.max_motor_speed^2
    w4 = Control.max_motor_speed^2;
end
if w4 < Control.min_motor_speed^2
    w4 = Control.min_motor_speed^2;
end

Control.O1 = sqrt(w1);    % Front M
Control.O2 = sqrt(w2);    % Right M
Control.O3 = sqrt(w3);    % Rear M
Control.O4 = sqrt(w4);    % Left M

% todo record control
% Control.O1_plot(Control.counter) = Control.O1;
% Control.O2_plot(Control.counter) = Control.O2;
% Control.O3_plot(Control.counter) = Control.O3;
% Control.O4_plot(Control.counter) = Control.O4;


%% Re-compute traditional control inputs

Control.U1 = Quad.KT*(Control.O1^2 + Control.O2^2 + Control.O3^2 + Control.O4^2);
% Control.U1_plot(Control.counter) = Control.U1;
  
Control.U2 = Quad.KT*Quad.l*(Control.O4^2 - Control.O2^2);
% Control.U2_plot(Control.counter) = Control.U2;
  
Control.U3 = Quad.KT*Quad.l*(Control.O1^2 - Control.O3^2);
% Control.U3_plot(Control.counter) = Control.U3;
  
Control.U4 = Quad.Kd*(Control.O1^2 + Control.O3^2 - Control.O2^2 - Control.O4^2);
% Control.U4_plot(Control.counter) = Control.U4;
  
Control.O = (Control.O1 - Control.O2 + Control.O3 - Control.O4);
% Control.O_plot(Control.counter) = Control.O;


end

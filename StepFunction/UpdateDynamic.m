function [DynamicStateParameterNew] = UpdateDynamic(ScenarioParameter,DynamicStateParameter,State,Control,Quad,time)
%UNTITLED17 Summary of this function goes here
%   Detailed explanation goes here

DynamicStateParameterNew = DynamicStateParameter;
X_dot_BF = State.C_B2I'*State.X_dot;
f_b_true = [(-Quad.Kdx*X_dot_BF(1))/Quad.m;...
            (-Quad.Kdy*X_dot_BF(2))/Quad.m;...
            (-Control.U1-Quad.Kdz*X_dot_BF(3))/Quad.m];


pqr = DynamicStateParameter.w_b;
if (ScenarioParameter.ScenarioMode == 0 )%static
    if (time>=50 && time<51)
    EulerDot = [0,0,60*pi/180]';
pqr = [1,1,-sin(State.theta);
       0,cos(State.phi),sin(State.phi)*cos(State.theta);
       0,-sin(State.phi),cos(State.phi)*cos(State.theta)]*EulerDot;
   DynamicStateParameterNew.w_b = pqr;
    else
        DynamicStateParameterNew.w_b = [0,0,0]';
    end
end

p = pqr(1);
q = pqr(2);
r = pqr(3);
w_b_dot_true = [(q*r*(Quad.Jy - Quad.Jz) - Quad.Jp*p*Control.Obar + Quad.l*Control.U2)/Quad.Jx;
(p*r*(Quad.Jz - Quad.Jx) + Quad.Jp*q*Control.Obar + Quad.l*Control.U3)/Quad.Jy;
(p*q*(Quad.Jx - Quad.Jy) + Control.U4)/Quad.Jz];
if (ScenarioParameter.ScenarioMode == 0 )%static
    f_b_true = State.C_B2I'*[0,0,-ScenarioParameter.g]';
    w_b_dot_true = zeros(3,1);
end
DynamicStateParameterNew.f_b = f_b_true;
DynamicStateParameterNew.w_b_dot = w_b_dot_true;
DynamicStateParameterNew.w_b = DynamicStateParameterNew.w_b + w_b_dot_true*ScenarioParameter.dt;
end


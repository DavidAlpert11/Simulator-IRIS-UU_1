function StateNew = UpdateState(ScenarioParameter,State,f_b,w_b)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here

StateNew = State;
% f_b=DynamicStateParameter.f_b;
% w_b=DynamicStateParameter.w_b;
f_n =State.C_B2I*f_b + [0,0,ScenarioParameter.g]';
StateNew.X_dot = State.X_dot + ScenarioParameter.dt*f_n;
StateNew.X = State.X + ScenarioParameter.dt*StateNew.X_dot;
StateNew.C_B2I = StateNew.C_B2I +StateNew.C_B2I*skew([w_b(1),w_b(2),w_b(3)])*ScenarioParameter.dt;
StateNew.C_B2I = (3/2)*StateNew.C_B2I-(1/2)*(StateNew.C_B2I*StateNew.C_B2I')*StateNew.C_B2I;
[ EulerAngles ] = dcm2euler( StateNew.C_B2I) ;
StateNew.phi = EulerAngles(1);
StateNew.theta = EulerAngles(2);
StateNew.psi = EulerAngles(3);%% Update Plotting Variables


end


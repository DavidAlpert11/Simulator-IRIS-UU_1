function NavStateNew = UpdateNavState(ScenarioParameter,NavState,f_b,w_b,IMUParameters)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here
NavStateNew = NavState;
% f_b=DynamicNavStateParameter.f_b;
% w_b=DynamicNavStateParameter.w_b;
f_n =NavState.C_B2I*f_b + [0,0,ScenarioParameter.g]';
NavStateNew.X_dot = NavState.X_dot + ScenarioParameter.dt*f_n;
NavStateNew.X = NavState.X + ScenarioParameter.dt*NavStateNew.X_dot;
NavStateNew.C_B2I = NavStateNew.C_B2I +NavStateNew.C_B2I*skew([w_b(1),w_b(2),w_b(3)])*ScenarioParameter.dt;
NavStateNew.C_B2I = (3/2)*NavStateNew.C_B2I-(1/2)*(NavStateNew.C_B2I*NavStateNew.C_B2I')*NavStateNew.C_B2I;
[ EulerAngles ] = dcm2euler( NavStateNew.C_B2I) ;
NavStateNew.phi = EulerAngles(1);
NavStateNew.theta = EulerAngles(2);
NavStateNew.psi = EulerAngles(3);%% Update Plotting Variables

%% gaus_markov

% b_a_dot = -NavStateNew.bias_acc./IMUParameters.tau_a + IMUParameters.sigma_acc_gm.*randn(3,1);
% b_g_dot = -NavStateNew.bias_gyro./IMUParameters.tau_g + IMUParameters.sigma_gyro_gm.*randn(3,1);
b_a_dot=0;
b_g_dot=0;
%%
NavStateNew.bias_acc = NavStateNew.bias_acc + b_a_dot*ScenarioParameter.dt;
NavStateNew.bias_gyro = NavStateNew.bias_gyro + b_g_dot*ScenarioParameter.dt;
% NavState = NavStateNew; 


end
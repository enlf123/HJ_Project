function dX=SEA_ode(t,X)
%% Set the variable
global no_load_speed no_load_current tor_constant 
global J_motor k_SEA d_SEA J_load
global Tor
q_m=X(1);
dq_m=X(2);
q_j=X(3);
dq_j=X(4);

Tor_j = k_SEA*(q_m-q_j)+d_SEA*(dq_m-dq_j);
ddq_m = (Tor -(Tor_j) - tor_constant*no_load_current/no_load_speed*dq_m)/J_motor;
ddq_j = Tor_j/J_load;


%% Set the result
dX(1)=dq_m;
dX(2)=ddq_m;
dX(3)=dq_j;
dX(4)=ddq_j;
dX=dX';


end
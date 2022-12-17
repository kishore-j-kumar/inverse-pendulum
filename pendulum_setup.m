%%% Inverted Pendulum Model 

%% Variable definition
M = 0.011; % (kg) mass of cart
m = 0.04429; % (kg) mass of pendulum
l = 0.200; % (m) length of pendulum
g = -9.81; % (m/s^2) gravity

%% State Definition

% See Latex documentation for state-space derivation

A = [0 1 0             0;
     0 0 m*g/M         0;
     0 0 0             1;
     0 0 (M+m)*g/(M*l) 0];

B = [0;
     1/M;
     0;
     1/(M*l)];

C = [1 0 0 0;
     0 0 1 0];

D = [0
     0];

states = {'x' 'x_dot' 'theta' 'theta_dot'};
inputs = {'F'};
outputs = {'x'; 'theta'};

inv_pendulum_ss = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

%% Modeling
Q = C'*C;
R = 1;
K = lqr(A,B,Q,R);

Ac = (A-B*K);

sys_cl = ss(Ac,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);

t = 0:0.01:5;
r =0.2*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t);
[AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','cart position (m)')
set(get(AX(2),'Ylabel'),'String','pendulum angle (radians)')
title('LQR Control')
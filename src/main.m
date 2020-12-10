close all
clear all
clc
%addpath('C:\Users\Olivier\Documents\EPFL 2020-2021\Model predictive control\casadi')
import casadi.*
% Test quad simulation: What does it do? Does it behave as expected?
quad = Quad();
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1);
u = [1;1;1;1];
% sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0);
% quad.plot(sim, u);

% Create trimmed and linearized system
quad = Quad();
[xs,us] = quad.trim(); % Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us); % Linearize the nonlinear model

% Decompose system in 4 independant systems
sys_transformed = sys * inv(quad.T); % New system is A * x + B * inv(T) * v
[sys_x,sys_y,sys_z,sys_yaw] = quad.decompose(sys_transformed, xs, us);

% Discretize the system 
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
sys_x_discrete = c2d(sys_x,Ts);
sys_y_discrete = c2d(sys_y,Ts);
sys_z_discrete = c2d(sys_z,Ts);
sys_yaw_discrete = c2d(sys_yaw,Ts);
%%
% Design MPC controller_x
% Compute controlled invariant set
% h = [0.035;0.035; 0; 0]; % [x, y, z, yaw]
% Hx = [0,1,0,0;
%       0 -1, 0 0];
% hx = [0.035;0.035];
% Gx = [1;-1];
% gx = [0.3;0.3]; 
% A = sys.A;  % [vel_pitch      pitch      vel_x          x] * [vel_pitch      pitch      vel_x          x]
% [nA, ~] = size(A);
% B = sys.B;  % [vel_pitch      pitch      vel_x          x] * u1
% [~, nB] = size(B);
% [Hxf,hxf] = Control_Invariant(Hx,hx,Gx,gx,A,B);
% [Hxt,hxt] = Terminal_Invariant(Hx,hx,Gx,gx,A,B);
%%
% Design MPC controllers
mpc.x = MPC_Control_x(sys_x, Ts);
mpc.y = MPC_Control_y(sys_y, Ts);
%%
mpc.z = MPC_Control_z(sys_z, Ts);
mpc.yaw = MPC_Control_yaw(sys_yaw, Ts);

%% Simulations (Linear)
% Get control inputs with
x0 = [0;0;0;2];
simTime = 10;
A = sys_x_discrete.A; B = sys_x_discrete.B;
xNext = x0;
xlist = [];
for k = 1:simTime/Ts
    ux = mpc.x.get_u(xNext);
    xNext = A*xNext+B*ux;
    xlist = [xlist,xNext];
end
plot(linspace(1,simTime,simTime/Ts),xlist(:,4))




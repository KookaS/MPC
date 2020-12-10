% load('src/handout/Quad.m')
addpath('/home/olivier/MATLAB/casadi-linux-matlabR2014b-v3.5.5')
import casadi.*
% Just testing git
quad = Quad();
Tf = 1.0; % Time to simulate for
x0 = zeros(12,1);
u = [1;1;1;1];
sim = ode45(@(t, x) quad.f(x, u), [0, Tf], x0);
quad.plot(sim, u);

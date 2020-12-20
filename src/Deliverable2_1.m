close all
clear all
clc
%%%%%%%%%%%%%%%% Deliverable 2.1 %%%%%%%%%%%%%%%%%%%%%%%
quad = Quad(); 
[xs,us] = quad.trim()
% Compute steady?state for which 0 = f(xs,us)
sys = quad.linearize(xs, us) % Linearize the nonlinear model
sys_transformed = sys * inv(quad.T) % New system is A * x + B * inv(T) * v
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us)

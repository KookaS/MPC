close all
clear all
clc
%%%%%%%%%%%%%% Simulations with the nonlinear drone model %%%%%%%%%%%%%%%
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc.x = MPC_Control_x(sys_x, Ts);
mpc.y = MPC_Control_y(sys_y, Ts);
mpc.z = MPC_Control_z(sys_z, Ts);
mpc.yaw = MPC_Control_yaw(sys_yaw, Ts);
sim = quad.sim(mpc.x, mpc.y, mpc.z, mpc.yaw);
quad.plot(sim);
close all
clear all
clc
%%%%%%%%%%%%%% Applying nonlinear MPC to our system %%%%%%%%%%%%%%%
quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL)
quad.plot(sim)

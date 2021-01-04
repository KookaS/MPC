close all
clear all
clc
%%%%%%%%%%%%%% Applying nonlinear MPC to our system %%%%%%%%%%%%%%%
quad = Quad();
CTRL = ctrl_NMPC(quad);
sim = quad.sim(CTRL)
quad.plot(sim)


% Plots
% 
% figure(1)
% saveas(gcf, 'Deliverable6_Figure1.png')
% 
% figure(2)
% view(90,0)
% saveas(gcf, 'Deliverable6_Figure2_yz.png')
% view(0,0)
% saveas(gcf, 'Deliverable6_Figure2_xz.png')
% view(0,90)
% saveas(gcf, 'Deliverable6_Figure2_yx.png')
% view(-45,45)
% saveas(gcf, 'Deliverable6_Figure2_3d.png')

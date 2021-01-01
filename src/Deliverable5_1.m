close all
clear all
clc

BIAS = 0.1;

Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);

Observability = obsv(sys_z.A, sys_z.C);

if rank(Observability) == size(Observability,1)
    
    
    mpc.x = MPC_Control_x(sys_x, Ts);
    mpc.y = MPC_Control_y(sys_y, Ts);
    mpc.z = MPC_Control_z(sys_z, Ts);
    mpc.yaw = MPC_Control_yaw(sys_yaw, Ts);
    
    sim = quad.sim(mpc.x, mpc.y, mpc.z, mpc.yaw, BIAS);
    quad.plot(sim);
    
    % Plots
    
    figure(1)
    saveas(gcf, 'Deliverable5_1_Figure1.png')
    
    figure(2)
    view(90,0)
    saveas(gcf, 'Deliverable5_1_Figure2_yz.png')
    view(0,0)
    saveas(gcf, 'Deliverable5_1_Figure2_xz.png')
    view(0,90)
    saveas(gcf, 'Deliverable5_1_Figure2_yx.png')
    view(-45,45)
    saveas(gcf, 'Deliverable5_1_Figure2_3d.png')
    
else
    
    error('Augmented System not Observable')

end

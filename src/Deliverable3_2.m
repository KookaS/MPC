close all
clear all
clc
%%%%%%%%%%%%%%%% Deliverable 3.2 %%%%%%%%%%%%%%%%%%%%%%%ù
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

%% Design MPC controllers
mpc.x = MPC_Control_x(sys_x, Ts);
mpc.y = MPC_Control_y(sys_y, Ts);
mpc.z = MPC_Control_z(sys_z, Ts);
mpc.yaw = MPC_Control_yaw(sys_yaw, Ts);

%%%%%%%%%% Simulations (Linear) %%%%%%%%%%%%%%%
%% Initializations
x0 = [0;0;0;0];y0 = [0;0;0;0];z0 = [0;0];yaw0 = [0;0];
xref = 1; yref = 1; zref = 2; yawref = pi/4;
simTime = 3;
Ax = sys_x_discrete.A; Bx = sys_x_discrete.B;
Ay = sys_y_discrete.A; By = sys_y_discrete.B;
Az = sys_z_discrete.A; Bz = sys_z_discrete.B;
Ayaw = sys_yaw_discrete.A; Byaw = sys_yaw_discrete.B;
xNext = x0;yNext = y0;zNext = z0;yawNext = yaw0;
xlist = [];ylist = [];zlist = [];yawlist = [];

%% Simulation
for k = 1:simTime/Ts
    ux = mpc.x.get_u(xNext,xref);
    uy = mpc.y.get_u(yNext,yref);
    uz = mpc.z.get_u(zNext,zref);
    uyaw = mpc.yaw.get_u(yawNext,yawref);
    xNext = Ax*xNext+Bx*ux;
    yNext = Ay*yNext+By*uy;
    zNext = Az*zNext+Bz*uz;
    yawNext = Ayaw*yawNext+Byaw*uyaw;
    xlist = [xlist,xNext];ylist = [ylist,yNext];zlist = [zlist,zNext];yawlist = [yawlist,yawNext];
end
%% Plots
figure
plot(linspace(1,simTime,simTime/Ts),xlist(4,:))
ylabel('X position')
figure
plot(linspace(1,simTime,simTime/Ts),ylist(4,:))
ylabel('Y position')
figure
plot(linspace(1,simTime,simTime/Ts),zlist(2,:))
ylabel('Z position')
figure
plot(linspace(1,simTime,simTime/Ts),yawlist(2,:))
ylabel('Yaw')


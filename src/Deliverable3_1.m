close all
clear all
clc
%%%%%%%%%%%%%%%% Deliverable 3.1 %%%%%%%%%%%%%%%%%%%%%%%ù
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
%% Initializations / Settling time of about 8 seconds
% Case 1
x0 = [0;0;0;2]; % two meters from the origin see task
y0 = [0;0;0;2]; % two meters from the origin see task
z0 = [0;2]; % two meters from the origin see task
yaw0 = [0;0]; % two meters from origin see task

% Setup of the Simulation
simTime = 10;
Ax = sys_x_discrete.A; Bx = sys_x_discrete.B;
Ay = sys_y_discrete.A; By = sys_y_discrete.B;
Az = sys_z_discrete.A; Bz = sys_z_discrete.B;
Ayaw = sys_yaw_discrete.A; Byaw = sys_yaw_discrete.B;
xNext = x0;yNext = y0;zNext = z0;yawNext = yaw0;
xlist = [];ylist = [];zlist = [];yawlist = [];

%Simulation
for k = 1:simTime/Ts
    ux = mpc.x.get_u(xNext);
    uy = mpc.y.get_u(yNext);
    uz = mpc.z.get_u(zNext);
    uyaw = mpc.yaw.get_u(yawNext);
    xNext = Ax*xNext+Bx*ux;
    yNext = Ay*yNext+By*uy;
    zNext = Az*zNext+Bz*uz;
    yawNext = Ayaw*yawNext+Byaw*uyaw;
    xlist = [xlist,xNext];ylist = [ylist,yNext];zlist = [zlist,zNext];yawlist = [yawlist,yawNext];
end

% Plots

figure(1), hold on
plot(linspace(1,simTime,simTime/Ts),xlist(4,:), 'DisplayName', 'Case 1', ...
    'LineWidth', 2)
ylabel('X position in [m]')
xlabel('Time in [s]')
title('Deliv. 3-1 X-Position over Time')

figure(2), hold on
plot(linspace(1,simTime,simTime/Ts),ylist(4,:), 'DisplayName', 'Case 1', ...
    'LineWidth', 2)
ylabel('Y position in [m]')
xlabel('Time in [s]')
title('Deliv. 3-1 Y-Position over Time')

figure(3), hold on
plot(linspace(1,simTime,simTime/Ts),zlist(2,:), 'DisplayName', 'Case 1', ...
    'LineWidth', 2)
ylabel('Z position in [m]')
xlabel('Time in [s]')
title('Deliv. 3-1 Z-Position over Time')

figure(4), hold on
plot(linspace(1,simTime,simTime/Ts),rad2deg(yawlist(2,:)), 'DisplayName', 'Case 1', ...
    'LineWidth', 2)
ylabel('Yaw in Degree')
xlabel('Time in [s]')
title('Deliv. 3-1 Yaw-Angle over Time')

%% Initializations / Settling time of about 8 seconds
% Case 2 
x0 = [0;0;0;0]; % yaw 45° see task
y0 = [0;0;0;0]; % yaw 45° see task
z0 = [0;0]; % yaw 45° see task
yaw0 = [0;pi/4]; % yaw 45° see task

% Setup of the Simulation
simTime = 10;
Ax = sys_x_discrete.A; Bx = sys_x_discrete.B;
Ay = sys_y_discrete.A; By = sys_y_discrete.B;
Az = sys_z_discrete.A; Bz = sys_z_discrete.B;
Ayaw = sys_yaw_discrete.A; Byaw = sys_yaw_discrete.B;
xNext = x0;yNext = y0;zNext = z0;yawNext = yaw0;
xlist = [];ylist = [];zlist = [];yawlist = [];

%Simulation
for k = 1:simTime/Ts
    ux = mpc.x.get_u(xNext);
    uy = mpc.y.get_u(yNext);
    uz = mpc.z.get_u(zNext);
    uyaw = mpc.yaw.get_u(yawNext);
    xNext = Ax*xNext+Bx*ux;
    yNext = Ay*yNext+By*uy;
    zNext = Az*zNext+Bz*uz;
    yawNext = Ayaw*yawNext+Byaw*uyaw;
    xlist = [xlist,xNext];ylist = [ylist,yNext];zlist = [zlist,zNext];yawlist = [yawlist,yawNext];
end

% Plots

figure(1)
plot(linspace(1,simTime,simTime/Ts),xlist(4,:), 'DisplayName', 'Case 2', ...
    'LineWidth', 2)
legend show
saveas(gcf, 'Deliverable3_1_x.png')

figure(2)
plot(linspace(1,simTime,simTime/Ts),ylist(4,:), 'DisplayName', 'Case 2', ...
    'LineWidth', 2)
legend show
saveas(gcf, 'Deliverable3_1_y.png')

figure(3)
plot(linspace(1,simTime,simTime/Ts),zlist(2,:), 'DisplayName', 'Case 2', ...
    'LineWidth', 2)
legend show
saveas(gcf, 'Deliverable3_1_z.png')

figure(4)
plot(linspace(1,simTime,simTime/Ts),rad2deg(yawlist(2,:)), 'DisplayName', 'Case 2', ...
    'LineWidth', 2)
legend show
saveas(gcf, 'Deliverable3_1_yaw.png')

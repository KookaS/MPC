%% Function to optimize Q and R matrix for every controller

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

step = 6;
i = 0;
token = 1000; % first criterion to compare

Eyaw = 1; % Velocity
Fyaw = 0; % Position
Lyaw = 0.01; % M_gamma

Ez = 1; % Velocity
Fz = 0; % Position
Lz = 0.01; % Lift

Ey = 0.01; % alpha velocity
Fy = 0.1; % alpha Position
Gy = 1; % y velocity
Hy = 0; % y Position
Ly = 0.01; % M_alpha

Ex = 0.01; % beta velocity
Fx = 0.1; % beta Position
Gx = 1; % x velocity
Hx = 0; % x Position
Lx = 0.01; % M_beta

% Yaw
for Fyaw = linspace(1,1000,step)
    
    Q_yaw = diag([0.1*Eyaw;Fyaw]);
    save('Q_yaw.mat', 'Q_yaw')
    R_yaw = 0.01 * Lyaw;
    save('R_yaw.mat', 'R_yaw')
    
    mpc.yaw = MPC_Control_yaw_opti(sys_yaw, Ts);
    
    % z - direction
    for Fz = linspace(1,1000,step)
        
        Q_z = diag([0.1*Ez;Fz]);
        save('Q_z.mat', 'Q_z')
        R_z = 0.01 * Lz;
        save('R_z.mat', 'R_z')
        
        mpc.z = MPC_Control_z_opti(sys_z, Ts);
        
        % y - direction
        for Hy = linspace(1,1000,step)
            
            Q_y = diag([Ey; Fy; Gy; Hy]);
            save('Q_y.mat', 'Q_y')
            R_y = 0.01 * Ly;
            save('R_y.mat', 'R_y')
            
            mpc.y = MPC_Control_y_opti(sys_y, Ts);
            
            % x - Direction
            for Hx = linspace(1,1000,step)
                
                Q_x = diag([Ex; Fx; Gx; Hx]);
                save('Q_x.mat', 'Q_x')
                R_x = 0.01 * Lx;
                save('R_x.mat', 'R_x')
                
                mpc.x = MPC_Control_x_opti(sys_x, Ts);
                
                i = i + 1
                
                %%%%%%%%%% Simulations (Linear) %%%%%%%%%%%%%%%
                %% Initializations / Settling time of about 8 seconds
                % Case 1 and 2 at the same time
                x0 = [0;0;0;2]; % two meters from the origin see task
                y0 = [0;0;0;2]; % two meters from the origin see task
                z0 = [0;2]; % two meters from the origin see task
                yaw0 = [0;pi/4]; % two meters from origin see task
                
                % Setup of the Simulation
                simTime = 8;
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
                
                xval = sum(xlist(4,:).^3);
                yval = sum(ylist(4,:).^3);
                zval = sum(zlist(2,:).^3);
                yawval = sum(yawlist(2,:).^3);
                
                crit = xval + yval + zval + yawval;
                
                if crit < token
                    close all
                    
                    QyawToken = Q_yaw;
                    RyawToken = R_yaw;
                    QzToken = Q_z;
                    RzToken = R_z;
                    QyToken = Q_y;
                    RyToken = R_y;
                    QxToken = Q_x;
                    RxToken = R_x;
                    
                    token = crit;
                    subplot(2,2,1), hold on
                    plot(linspace(0,simTime,simTime/Ts),xlist(4,:), 'DisplayName', 'Case 1', ...
                        'LineWidth', 2)
                    ylabel('X position in [m]')
                    xlabel('Time in [s]')
                    title('Deliv. 3-1 X-Position over Time')
                    
                    subplot(2,2,2), hold on
                    plot(linspace(0,simTime,simTime/Ts),ylist(4,:), 'DisplayName', 'Case 1', ...
                        'LineWidth', 2)
                    ylabel('Y position in [m]')
                    xlabel('Time in [s]')
                    title('Deliv. 3-1 Y-Position over Time')
                    
                    subplot(2,2,3), hold on
                    plot(linspace(0,simTime,simTime/Ts),zlist(2,:), 'DisplayName', 'Case 1', ...
                        'LineWidth', 2)
                    ylabel('Z position in [m]')
                    xlabel('Time in [s]')
                    title('Deliv. 3-1 Z-Position over Time')
                    
                    subplot(2,2,4), hold on
                    plot(linspace(0,simTime,simTime/Ts),rad2deg(yawlist(2,:)), 'DisplayName', 'Case 1', ...
                        'LineWidth', 2)
                    ylabel('Yaw in Degree')
                    xlabel('Time in [s]')
                    title('Deliv. 3-1 Yaw-Angle over Time')
                end
            end
        end
    end
end


QyawToken
RyawToken
QzToken
RzToken
QyToken
RyToken
QxToken
RxToken
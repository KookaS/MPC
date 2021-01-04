%% Plots for Dev 6
% close all
% clear all
% clc

normal = load('Sim_normal.mat');

soft = load('Sim_soft.mat');

soft = load('Sim_nonTS.mat');


time = zeros(1,40*5);
xpos_norm = zeros(1,40*5);
xpos_soft = zeros(1,40*5);
ypos_norm = zeros(1,40*5);
ypos_soft = zeros(1,40*5);
zpos_norm = zeros(1,40*5);
zpos_soft = zeros(1,40*5);

roll_norm = zeros(1,40*5);
roll_soft = zeros(1,40*5);
pitch_norm  = zeros(1,40*5);
pitch_soft = zeros(1,40*5);
yaw_norm = zeros(1,40*5);
yaw_soft = zeros(1,40*5);

for i = 1:40*5
    
    time(i) = normal.sim(i).t;
    
    xpos_norm(i) = normal.sim(i).pos(1,1);
    xpos_soft(i) = soft.sim(i).pos(1,1);
    xref(i) = normal.sim(i).ref(1,1);
    ypos_norm(i) = normal.sim(i).pos(2,1);
    ypos_soft(i) = soft.sim(i).pos(2,1);
    yref(i) = normal.sim(i).ref(2,1);
    zpos_norm(i) = normal.sim(i).pos(3,1);
    zpos_soft(i) = soft.sim(i).pos(3,1);
    zref(i) = normal.sim(i).ref(3,1);
    
    roll_norm(i) = normal.sim(i).theta(1,1);
    roll_soft(i) = soft.sim(i).theta(1,1);
    pitch_norm(i) = normal.sim(i).theta(2,1);
    pitch_soft(i) = soft.sim(i).theta(2,1);
    yaw_norm(i) = normal.sim(i).theta(3,1);
    yaw_soft(i) = soft.sim(i).theta(3,1);
    
end

figure(1), hold on
plot(time, xpos_norm, 'Color', 'b', 'LineStyle', '--', 'DisplayName', 'x-Pos normal')
plot(time, xpos_soft, 'Color', 'b', 'DisplayName', 'x-Pos soft', 'Marker', 'o')
plot(time, ypos_norm, 'Color', 'r', 'LineStyle', '--', 'DisplayName', 'y-Pos normal')
plot(time, ypos_soft, 'Color', 'r', 'DisplayName', 'y-Pos soft', 'Marker', 'o')
plot(time, zpos_norm, 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '--', 'DisplayName', 'z-Pos normal')
plot(time, zpos_soft, 'Color', [0.9290 0.6940 0.1250], 'DisplayName', 'z-Pos soft', 'Marker', 'o')

plot(time, xref, 'Color', 'k', 'DisplayName', 'x-Ref')
plot(time, yref, 'Color', 'k', 'DisplayName', 'y-Ref')
plot(time, zref, 'Color', 'k', 'DisplayName', 'z-Ref')

legend show
legend('Location', 'northwest')
xlabel('Time in [s]')
ylabel('Position in [m]')
title('Comparison of soft / normal Constraints Position')
saveas(gcf, 'Deliverable6_Comparison_Pos.png')

figure(2), hold on
plot(time, rad2deg(roll_norm), 'Color', 'b', 'LineStyle', '--', 'DisplayName', 'roll normal')
plot(time, rad2deg(roll_soft), 'Color', 'b', 'DisplayName', 'roll soft', 'Marker', 'o')
plot(time, rad2deg(pitch_norm), 'Color', 'r', 'LineStyle', '--', 'DisplayName', 'pitch normal')
plot(time, rad2deg(pitch_soft), 'Color', 'r', 'DisplayName', 'pitch soft', 'Marker', 'o')
plot(time, rad2deg(yaw_norm), 'Color', [0.9290 0.6940 0.1250], 'LineStyle', '--', 'DisplayName', 'yaw normal')
plot(time, rad2deg(yaw_soft), 'Color', [0.9290 0.6940 0.1250], 'DisplayName', 'yaw soft', 'Marker', 'o')

legend show
ldg = legend('Location', 'northwest', 'Orientation', 'horizontal');
ldg.NumColumns = 2;
xlabel('Time in [s]')
ylabel('Angle in [Degree]')
title('Comparison of soft / normal Constraints Angles')
saveas(gcf, 'Deliverable6_Comparison_Angle.png')


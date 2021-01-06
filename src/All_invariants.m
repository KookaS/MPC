function [Hfx,hfx,Qfx,Hfy,hfy,Qfy,Hfz,hfz,Qfz,Hfyaw,hfyaw,Qfyaw] =  All_invariants()
Ts = 1/5;
quad = Quad(Ts);
[xs, us] = quad.trim();
sys = quad.linearize(xs, us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
mpc.x = MPC_Control_x(sys_x, Ts);
mpc.z = MPC_Control_z(sys_z, Ts);
mpc.yaw = MPC_Control_yaw(sys_yaw, Ts);
% Constraints for x and y
Hx = [0,1,0,0;
0 -1, 0 0];
hx = [0.035;0.035];
Gx = [1;-1];
gx = [0.3;0.3]; 
Ax = mpc.x.A; [nAx, ~] = size(Ax);
Bx = mpc.x.B; [~, nBx] = size(Bx);
[Kx,Qfx] = dlqr(Ax,Bx,eye(nAx),eye(nBx)); Kx = -Kx;
[Hfx,hfx] = Terminal_Invariant(Hx,hx,Gx,gx,Ax,Bx,Kx);
Hfy = Hfx; hfy = hfx; Qfy = Qfx;

% Constraints for z
Gz = [1;-1]; gz = [0.3;0.2];
Hz = [0,0];
hz = 0;
Az = mpc.z.A; [nAz, ~] = size(Az);
Bz = mpc.z.B; [~, nBz] = size(Bz);
[Kz,Qfz] = dlqr(Az,Bz,eye(nAz),eye(nBz)); Kz = -Kz;
[Hfz,hfz] = Terminal_Invariant(Hz,hz,Gz,gz,Az,Bz,Kz);

% Constraints for yaw
Gyaw = [1;-1]; gyaw = [0.2;0.2];
Hyaw = [0,0]; hyaw = 0;
Ayaw = mpc.yaw.A; [nAyaw, ~] = size(Ayaw);
Byaw = mpc.yaw.B; [~, nByaw] = size(Byaw);
[Kyaw,Qfyaw] = dlqr(Ayaw,Byaw,eye(nAyaw),eye(nByaw)); Kyaw = -Kyaw;
[Hfyaw,hfyaw] = Terminal_Invariant(Hyaw,hyaw,Gyaw,gyaw,Ayaw,Byaw,Kyaw);

end


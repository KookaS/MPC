
function [ctrl, traj] = ctrl_NMPC(quad)
import casadi.*
opti = casadi.Opti(); % Optimization problem
N = 20; % MPC horizon [SET THIS VARIABLE]
% ???? decision variables ?????????
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)
X0 = opti.parameter(12,1); % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
XREF = opti.parameter(12,1); % Reference state
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% YOUR CODE HERE %%%%
% Create Runge Kutta function handle
f = @(x,u) quad.f(x,u);
h = quad.Ts;
F = @(x,u) RK4(x,u,h,f);

%  Compute constraints and add them to optimizer
H = zeros(4,12); H(1,4) = 1; H(2,4) = -1; H(3,5) = 1; H(4,5) = -1;
h = 0.035*ones(4,1); % State constraints (only present on roll and pitch)
G = [eye(4);-eye(4)]; 
g = [1.5*ones(4,1);zeros(4,1)]; % Input constraints 
C = zeros(12,4); C(6,1) = 1; C(10,2) = 1; C(11,3) = 1; C(12,4) = 1; 
% Add constraints to optimizer
for i=1:N
 opti.subject_to(X(:,i+1) == F(X(:,i),U(:,i))); % Dynamics
 opti.subject_to(H*X(:,i)<=h); % State constraints
 opti.subject_to(G*U(:,i)<=g); % Input constraints
end
opti.subject_to(H*X(:,N+1)<=h);
% Define cost function
Qrotation = 0.1*eye(3); Qangle = 1*eye(3); Qspeed = 0.1*eye(3); Qposition = 100*eye(3);
R = 10^-5*eye(4);
Q = blkdiag(Qrotation,Qangle,Qspeed,Qposition);
Qf = Q;
cost = 0;
for i=1:N
    cost = cost+(X(:,i)-XREF)'*Q*(X(:,i)-XREF)+U(:,i)'*R*U(:,i);
end
cost = cost+(X(:,N+1)-XREF)'*Qf*(X(:,N+1)-XREF);
opti.minimize(cost);

%%%%%%%%%%%%%%%%%%%%%%%%
ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF,XREF, X, U);
end
function u = eval_ctrl(x, ref, opti, X0, REF,XREF,X, U)
% ???? Set the initial state and reference ????
opti.set_value(X0, x);
opti.set_value(REF, ref);
C = zeros(12,4); C(6,1) = 1; C(10,2) = 1; C(11,3) = 1; C(12,4) = 1; 
opti.set_value(XREF, C*ref);
% ???? Setup solver NLP ??????
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
opti.solver('ipopt', ops);
% ???? Solve the optimization problem ????
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));
% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
%opti.set_initial(opti.lam.g, sol.value(opti.lam.g));
end



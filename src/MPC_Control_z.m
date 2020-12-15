classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection
    L                   % Estimator gain for disturbance rejection
  end

  methods
    function mpc = MPC_Control_z(sys, Ts)
      display('supposed to update here')
      mpc = mpc@MPC_Control(sys, Ts);
      display('supposed to update here')
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end

    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);

      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);

      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 20;

      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);


      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      % No state constraints for z
      % Input constraints are

      G = [1;-1]; g = [0.3;0.2];
      H = [0,0];
      h = 0;
      % Compute (Choose) cost functions
      Q = diag([0.5;10]); R = 0.1*eye(1);
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;

      % Regulation to the origin for 3.2
%       [K,Qf] = dlqr(A,B,eye(nA),eye(nB)); K = -K;
%       [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K);
%       for i = 1:N-1
%       con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%       con = [con, G*u(i) <= g]; % Input constraints
%       obj = obj+x(:,i)'*Q*x(:,i)+u(i)'*R*u(i);
%       end
%       obj = obj+x(:,N)'*Qf*x(:,N);
%       con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints

      % Reference tracking for 4.1
%       for i = 1:N-1
%           con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%           con = [con, G*u(i) <= g]; % Input constraints
%           obj = obj+(x(:,i)-xs)'*Q*(x(:,i)-xs)+(u(i)-us)'*R*(u(i)-us);
%       end

        % Reference tracking for 5.1
        for i = 1:N-1
            x(:,i+1) = mpc.A*x(:,i) + mpc.B*u(i);
            obj = obj+(x(:,i+1)-xs)'*Q*(x(:,i+1)-xs)+(u(i)-us)'*R*(u(i)-us);
            con = [con,G*u(i)<=g];
        end

      obj = obj+x(:,N)'*Q*x(:,N);

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end


    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;

      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;

      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      G = [1;-1]; g = [0.3;0.2];
      con = [con,xs == mpc.A*xs+mpc.B*us];
      con = [con,mpc.C*xs + d_est ==ref];    % added d_est
      con = [con,G*us<=g];
      obj = us'*us;

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
    end


    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)

      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D

        A = mpc.A; [nA, ~] = size(A);
        B = mpc.B; [~, nB] = size(B);
        C = mpc.C; [nC, ~] = size(C);
        D = mpc.D;
        A_bar = [A, B;
                zeros(1,nA),1];
        B_bar = [B;zeros(1,nB)];
        C_bar = [C,ones(nC,1)];

        % the smaller the poles, the bigger the gain, the less it falls at the beginning
%        L = -place(A_bar',C_bar',[0.6,0.7,0.8])';
         L = -place(A_bar',C_bar',[0.01,0.02,0.03])';

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end


  end
end

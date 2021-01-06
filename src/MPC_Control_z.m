classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection
    L                   % Estimator gain for disturbance rejection
  end

  methods
    function mpc = MPC_Control_z(sys, Ts)
      %display('supposed to update here')
      mpc = mpc@MPC_Control(sys, Ts);
      % Setting up estimator
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
      %display('supposed to update here')

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
      % Setting up estimator
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
      % SET THE HORIZON HERE
      N = 30;

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
      Q = diag([1;10]); R = 0.01*eye(1);
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
        A = mpc.A; [nA, ~] = size(A);
        B = mpc.B; [~, nB] = size(B);
      % Regulation to the origin for 3.1
       [K,Qf] = dlqr(A,B,eye(nA),eye(nB)); K = -K;
       [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K, 'z');
%       for i = 1:N-1
%       con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%       con = [con, G*u(i) <= g]; % Input constraints
%       obj = obj+x(:,i)'*Q*x(:,i)+u(i)'*R*u(i);
%       end
%       obj = obj+x(:,N)'*Qf*x(:,N);
%       con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints

      % Reference tracking for 3.2
%       for i = 1:N-1
%           con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%           con = [con, G*u(i) <= g]; % Input constraints
%           obj = obj+(x(:,i)-xs)'*Q*(x(:,i)-xs)+(u(i)-us)'*R*(u(i)-us);
%       end
%       obj = obj+x(:,N)'*Qf*x(:,N);
%       con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints
        % Reference tracking for 5.1
        for i = 1:N-1
          con = [con, mpc.A*x(:,i)+mpc.B*u(i)+mpc.B*d_est ==  x(:,i+1)]; % System dynamics
          con = [con, G*u(i) <= g]; % Input constraints
          obj = obj+(x(:,i)-xs)'*Q*(x(:,i)-xs)+(u(i)-us)'*R*(u(i)-us);
        end
      obj = obj+x(:,N)'*Qf*x(:,N); % Terminal Cost
      con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints

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
      % Setting up estimator
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      G = [1;-1]; g = [0.3;0.2];
      Aex = [eye(2)-mpc.A, -mpc.B;
             mpc.C, zeros(1,1)];
      con = [con,Aex*[xs;us] == [mpc.B*d_est;ref]];  %[zeros(2,1);ref]]; %Use commented for no offset free tracking
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

        A = mpc.A; [~, nA] = size(A);
        B = mpc.B; [~, nB] = size(B);
        C = mpc.C; [~, nC] = size(C);
        D = mpc.D;
        Bd = B;
        A_bar = [A, Bd;
                zeros(1,nA),ones(1,nB)]; % 1 row since disturbance is scalar
        B_bar = [B; zeros(1,nB)]; % 1 row since disturbance is scalar
        C_bar = [C, zeros(1,1)]; % (1,1) since disturbance is scalar
        
        if rank([A-diag([1,1]), B; C, zeros(1,nB)]) < 3 % full collum rank ...
            %of matrix
            
            error('Augmented System is not Observable')
            
        end
        % the smaller the poles, the bigger the gain, the less it falls at the beginning
         L = -place(A_bar',C_bar',[0.05,0.06,0.07])';
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end


  end
end

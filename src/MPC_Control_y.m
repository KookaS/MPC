classdef MPC_Control_y < MPC_Control

  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);

      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);

      % SET THE HORIZON HERE
      N = 30;

      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);


      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
% Compute invariant sets
    H = [0,1,0,0;
    0 -1, 0 0];
    h = [0.035;0.035];
    G = [1;-1];
    g = [0.3;0.3];
    A = mpc.A; [nA, ~] = size(A);
    B = mpc.B; [~, nB] = size(B);
    [K,Qf] = dlqr(A,B,eye(nA),eye(nB)); K = -K;
    %[Hf,hf] = Control_Invariant(H,h,G,g,A,B);
    [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K,'y');
    % Compute (Choose) cost functions
    Q = diag([0.01;0.01;1;200.8]); R = 0.001*eye(1);
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      % Regulation to origin
%       for i = 1:N-1
%       con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%       con = [con, H*x(:,i)<=h]; % State constraints
%       con = [con, G*u(i) <= g]; % Input constraints => I think these are not necessary as this
%                                  %is already included in the control invariant set
%       obj = obj+x(:,i)'*Q*x(:,i)+u(i)'*R*u(i);
%       end
%       con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints
%       obj = obj+x(:,N)'*Qf*x(:,N);

      % Reference tracking
      for i = 1:N-1
      con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
      con = [con, H*(x(:,i))<=h]; % State constraints
      con = [con, G*(u(i)) <= g]; % Input constraints
      obj = obj+(x(:,i)-xs)'*Q*(x(:,i)-xs)+(u(i)-us)'*R*(u(i)-us);
      end
      con = [con,Ht*(x(:,N))<=ht]; % Terminal state constraints
      obj = obj+(x(:,N)-xs)'*Qf*(x(:,N)-xs);


      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end


    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;

      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
            H = [0,1,0,0;
        0 -1, 0 0];
        h = [0.035;0.035];
        G = [1;-1];
        g = [0.3;0.3];
      con = [con,xs == mpc.A*xs+mpc.B*us];
      con = [con,mpc.C*xs==ref];
      con = [con,H*xs<=h];
      con = [con,G*us<=g];
      obj = us'*us;

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});

    end
  end
end

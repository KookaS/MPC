classdef MPC_Control_yaw < MPC_Control
  
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
      N = 20;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      % No state constraints for yaw
      % Input constraints are
      G = [1;-1]; g = [0.2;0.2];
      H = [0,0]; h = 0;
      A = mpc.A; [nA, ~] = size(A);
      B = mpc.B; [~, nB] = size(B);
      [K,Qf] = dlqr(A,B,eye(nA),eye(nB)); K = -K;
      [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K);
      % Compute (Choose) cost functions
      Q = diag([1;10]); R = 0.01*eye(1); 
      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
%       for i = 1:N-1
%       con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
%       con = [con, G*u(i) <= g]; % Input constraints 
%       obj = obj+x(:,i)'*Q*x(:,i)+u(i)'*R*u(i);
%       end
%       obj = obj+x(:,N)'*Qf*x(:,N);
%       con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints

      % Reference tracking
      for i = 1:N-1
      con = [con, mpc.A*x(:,i)+mpc.B*u(i) ==  x(:,i+1)]; % System dynamics
      con = [con, G*u(i) <= g]; % Input constraints 
      obj = obj+(x(:,i)-xs)'*Q*(x(:,i)-xs)+(u(i)-us)'*R*(u(i)-us);
      end
      obj = obj+x(:,N)'*Qf*x(:,N);
      con = [con,Ht*x(:,N)<=ht]; % Terminal state constraints
      
      
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
      % Regulation to the origin
      con = [];
      G = [1;-1]; g = [0.2;0.2];
      con = [con,xs == mpc.A*xs+mpc.B*us];
      con = [con,mpc.C*xs==ref];
      con = [con,G*us<=g];
      obj = us'*us;
      

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end

% Some testing for the nonlinear MPC implementation
quad = Quad();
h = 1/2;
f = @(x,u) quad.f(x,u);
F = @(x,u) RK4(x,u,h,f);
X = zeros(12,1); U = 0.707*ones(4,1);
F(X,U)
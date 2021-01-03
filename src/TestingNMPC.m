% Some testing for the nonlinear MPC implementation
quad = Quad();
h = 1/2;
f = @(x,u) quad.f(x,u);
F = @(x,u) RK4(x,u,h,f);
X = zeros(12,1); U = 0.7007*ones(4,1);
for i=1:20
X = F(X,U)
end
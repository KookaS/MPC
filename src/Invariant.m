function [Hf,hf] = Invariant(H,h, Hv, hv,sys)

A = sys.A;  % [vel_pitch      pitch      vel_x          x] * [vel_pitch      pitch      vel_x          x]
[nA, ~] = size(A);
B = sys.B;  % [vel_pitch      pitch      vel_x          x] * u1
[~, nB] = size(B);

% Compute LQR controller
K = dlqr(A,B,eye(nA),eye(nB));
K = -K; % Note that matlab defines K as -K
Ak = A+B*K; % Closed-loop dynamics

display(H)
display(K)
display(Hv)

HH = [H;(Hv*K).'];
hh = [h;hv]; % State and input constraints

display(HH)
display(hh)
preW = polytope(HH,hh);
W = preW;

figure();
hold on
i = 1;
while 1 && i<100
    %preW = polytope([H;H*Ak],[h;h]);
    [F,f] = double(preW);	
	% Compute the pre-set
    display(F)
    display(f)
    display(Ak)
    display([F;F*Ak])
    display([f;f])

	preW = polytope([F;F*Ak],[f;f]);
    Intersect = intersect(preW, W);
    
    W.projection(1:2).plot('color','g','alpha', 0.2);
    preW.projection(1:2).plot('color','c','alpha', 0.2);
    Intersect.projection(1:2).plot('color','y','alpha', 1);
    axis square
   
    if Intersect == W
        break
    else
        i = i+1;
        W = Intersect;
        W = W.minHRep;
        H = W.A;
        h = W.b;
    end
end
hold off
fprintf('Maximal control invariant set computed after %i iterations\n\n', i);
Hf = H;
hf = h;
end
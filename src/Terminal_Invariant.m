function [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B)

[nA, ~] = size(A);
[~,nB] = size(B);
% Compute LQR controller
K = dlqr(A,B,eye(nA),eye(nB));
K = -K; % Note that matlab defines K as -K
Amod = A+B*K;
Hmod = [H;G*K];
hmod = [h;g];
W = Polyhedron(Hmod,hmod);
i = 1;

%% Iterate to find invariant set
while 1 && i<20
    preW = Polyhedron(Hmod*Amod,hmod);
    Intersect = intersect(preW, W);
%     figure
%     hold on
%     W.projection(1:2).plot('color', 'y','alpha', 0.8);
%     preW.projection(1:2).plot('color','c','alpha', 0.2);
%     Intersect.projection(1:2).plot('color','r','alpha', 0.5);
%     axis square
%     hold off
    if Intersect == W
        break
    else
        i = i+1
        W = Intersect;
        Hmod = W.A;
        hmod = W.b;
    end

end
Ht = Hmod;
ht = hmod;
% display(HH)
% display(hh)
% preW = polytope(HH,hh);
% W = preW;
% 
% figure();
% hold on
% i = 1;
% while 1 && i<100
%     %preW = polytope([H;H*Ak],[h;h]);
%     [F,f] = double(preW);	
% 	% Compute the pre-set
%     display(F)
%     display(f)
%     display(Ak)
%     display([F;F*Ak])
%     display([f;f])
% 
% 	preW = polytope([F;F*Ak],[f;f]);
%     Intersect = intersect(preW, W);
%     
%     W.projection(1:2).plot('color','g','alpha', 0.2);
%     preW.projection(1:2).plot('color','c','alpha', 0.2);
%     Intersect.projection(1:2).plot('color','y','alpha', 1);
%     axis square
%    
%     if Intersect == W
%         break
%     else
%         i = i+1;
%         W = Intersect;
%         W = W.minHRep;
%         H = W.A;
%         h = W.b;
%     end
% end
% hold off
% fprintf('Maximal control invariant set computed after %i iterations\n\n', i);
% Hf = H;
% hf = h;
% end


%% Iterate to find invariant set
while 1 && i<10

    preW = Polyhedron(Hnew,hs); % heqr is the mistake
    Intersect = intersect(preW, W);
    figure
    hold on
    W.plot('color', 'y','alpha', 0.2);
    preW.plot('color','c','alpha', 0.2);
    Intersect.plot('alpha', 1);
    axis square
    hold off
    if Intersect == W
        break
    else
        i = i+1
        W = Intersect;
        Hnew = W.A;
        horProjection = W.projection(1:2);
        Fnew = horProjection.A;
        fnew = horProjection.b;
        Hnew = [Fnew*A Fnew*B;
            zeros(2) G];
        hs = [fnew; g];
    end

end
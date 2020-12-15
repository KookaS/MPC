function [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K)

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
        i = i+1;
        W = Intersect;
        Hmod = W.A;
        hmod = W.b;
    end

end
Ht = Hmod;
ht = hmod;

function [Hf,hf] = Invariant(H,h,A)
i = 1;
P = Polyhedron(H,h);
W = P;
while 1 && i<100
    preW = Polyhedron(H*A,h);
    Intersect = intersect(preW, W);
    figure
    hold on
    W.projection(1:2).plot('color','g','alpha', 0.2);
     preW.projection(1:2).plot('color','c','alpha', 0.2);
     Intersect.projection(1:2).plot('color','y','alpha', 1);
    axis square
    hold off
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
Hf = H;
hf = h;
end


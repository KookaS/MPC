function [Hf,hf] = Control_Invariant(H,h,G,g,A,B)

Hs = [H,zeros(2,1);
      zeros(2,4) G];
hs = [h;g];
W = Polyhedron(Hs,hs);
Hnew = [H*A H*B;
        zeros(2,4) G];
i = 1;
%% Iterate to find invariant set
while 1 && i<10

    preW = Polyhedron(Hnew,hs); % heqr is the mistake
    Intersect = intersect(preW, W);
    figure
    hold on
     W.projection(1:2).plot('color', 'y','alpha', 0.9);
     preW.projection(1:2).plot('color','c','alpha', 0.2);
     %Intersect.projection(1:2).plot('color','r','alpha', 0.5);
    axis square
    hold off
    if Intersect == W
        break
    else
        i = i+1
        W = Intersect;
        horProjection = W.projection(1:4);
        Fnew = horProjection.A;
        fnew = horProjection.b;
        Hnew = [Fnew*A Fnew*B;
            zeros(2,4) G];
        hs = [fnew; g];
    end

end
Hf = Fnew;
hf = fnew;
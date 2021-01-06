function [Ht,ht] = Terminal_Invariant(H,h,G,g,A,B,K, name)

Amod = A+B*K;
Hmod = [H;G*K];
hmod = [h;g];
W = Polyhedron(Hmod,hmod);
i = 1;

%% Iterate to find invariant set
while 1 && i<20
    preW = Polyhedron(Hmod*Amod,hmod);
    Intersect = intersect(preW, W);
    
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

% Plot
% 
% if strcmp(name,'z') == 1 || strcmp(name,'yaw') == 1
%     
%     figure , hold on, grid on
%     preW.projection(1:2).plot('color','c','alpha', 0.2);
%     W.projection(1:2).plot('color', 'y','alpha', 1);
% 
%     axis square
%     title(['Invariant Set and Preset Set for ', name, '-Coordiante'], ...
%         'FontSize', 13)
%     legend('Invariant Set', 'Preset of Invariant Set')
%     hold off
%     saveas(gcf, ['Deliverable3_1_TS_', name, '.png']) 
%     
% else
%     
%     figure , hold on, grid on
%     preW.projection(1:2).plot('color','c','alpha', 0.2);
%     W.projection(1:2).plot('color', 'y','alpha', 1);
%     axis square
%     title(['Projection 1:2 of Invariant Set and Preset Set for ', name, ...
%         '-Coordiante'], 'FontSize', 13)
%     legend('Invariant Set', 'Preset of Invariant Set')
%     hold off
%     saveas(gcf, ['Deliverable3_1_TS12_', name, '.png'])
%     
%     figure, hold on, grid on
%     preW.projection(2:3).plot('color','c','alpha', 0.2);
%     W.projection(2:3).plot('color', 'y','alpha', 1);
%     axis square
%     title(['Projection 2:3 of Invariant Set and Preset Set for ', name, ...
%         '-Coordiante'], 'FontSize', 13)
%     hold off
%     saveas(gcf, ['Deliverable3_1_TS23_', name, '.png'])
%     
%     figure, hold on, grid on
%     preW.projection(3:4).plot('color','c','alpha', 0.2);
%     W.projection(3:4).plot('color', 'y','alpha', 1);
%     axis square
%     title(['Projection 3:4 of Invariant Set and Preset Set for ', name, ...
%         '-Coordiante'], 'FontSize', 13)
%     hold off
%     saveas(gcf, ['Deliverable3_1_TS34_', name, '.png'])
% end
close all


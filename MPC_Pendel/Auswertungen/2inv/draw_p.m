figure(1); clf;
axis manual
axis equal;
ylim([-0.7 0.7]);
xlim([-1.0 1.0]);
h = hgtransform;

% rectangle Position Fusspunkt1, Fusspunkt2, Breite, Höhe
rectangle('Position',[-0.51,-0.03,0.06,0.06],'Curvature',[1,1],'FaceColor',[0.8,0.8,0.8],'Parent',h);   % Abrundung
rectangle('Position',[0.45,-0.03,0.06,0.06],'Curvature',[1,1],'FaceColor',[0.8,0.8,0.8],'Parent',h);    % Abrundung

% Schiene
h2 = rectangle('Position',[-0.48,-0.03,0.96,0.06],'FaceColor',[0.8,0.8,0.8],'Parent',h);
h2.LineStyle='none';
h2 = rectangle('Position',[-0.44,0.02,0.88,0.01],'FaceColor','white','Parent',h);
h2.LineStyle='none';

% Schwarze Umrandung
plot([-0.48,-0.44,-0.44,0.44,0.44,0.48],[0.03,0.03,0.02,0.02,0.03,0.03],'k','Parent',h);    %oben
plot([-0.48,0.48],[-0.03,-0.03],'k','Parent',h);   % unten
plot(-0.48,0,'k.','LineWidth',3,'Parent',h);    % Punkt links
plot(0.48,0,'k.','LineWidth',3,'Parent',h);     % Punkt rechts

h.Matrix = makehgtform('translate',[0 -0.04 0]); % Verschiebung der Schiene nach unten



l1 = 0.17;
l2 = 0.314;

% Wagen
h2 = hgtransform;
rectangle('Position',[-0.05,-0.02,0.1,0.04],'FaceColor',[0.8,0.8,0.8],'Parent',h2);

% Stab 1
h3 = hgtransform('Parent',h2);
rectangle('Position',[-0.01,-0.16,0.02,0.17],'FaceColor',[0.95,0.95,0.95],'Parent',h3);
plot(0,0,'k.','LineWidth',3,'Parent',h2);   % Verbindungspunkt 1
plot(0,-0.165,'k.','LineWidth',3,'Parent',h3);   % Verbindungspunkt 2

% Stab 2
h4 = hgtransform('Parent',h3);
rectangle('Position',[-0.01,-0.314,0.02,0.314],'FaceColor',[0.95,0.95,0.95],'Parent',h4);


% % alpha = pi/2;
% % alpha2 = pi;
% % alpha3 = alpha2 - alpha;
% % h2.Matrix = makehgtform('translate',[0 0 0]);
% % pause(1);
% % h3.Matrix = makehgtform('zrotate',pi - alpha);
% % pause(1);
% % h4.Matrix = makehgtform('translate',[0 -0.17 0],'zrotate' ,pi - alpha2);
% % pause(1);
% % 
% % h4.Matrix = makehgtform('translate',[l1*cos(pi - alpha) l1*sin(pi - alpha) 0],'zrotate',pi - alpha2);
% % 
% % h4.Matrix = makehgtform('translate',[0.17*sin(pi-alpha) 0.17*cos(pi-alpha) 0])*makehgtform('zrotate',pi/2);
% % pause(1);
% % h4.Matrix = makehgtform('translate',[0 -0.17 0]);
% % pause(1);
% % 

% 
% alpha = -pi/4;
% alpha2 = -pi/3;
% 
% h3.Matrix = makehgtform('zrotate',abs(alpha-pi));
% h4.Matrix = makehgtform('translate',[0 -0.17 0],'zrotate' ,-abs(alpha-pi),'zrotate' ,abs(alpha2-pi));
% pause(1);

% Winkel in der linken Hälfte negativ eintragen also Einträge in Datei
% umwandeln, Minus 2 Pi also, falls Winkel > Pi

alpha = pi/2;
alpha2 = -pi/3;
x = 0.35;
if alpha > pi
    alpha = alpha - 2*pi;
end

if alpha2 > pi
    alpha2 = alpha2 -2*pi;
end

h2.Matrix = makehgtform('translate',[x 0 0]);
h3.Matrix = makehgtform('zrotate',abs(alpha-pi));
h4.Matrix = makehgtform('translate',[0 -0.17 0],'zrotate' ,-abs(alpha-pi),'zrotate' ,abs(alpha2-pi));
pause(1);








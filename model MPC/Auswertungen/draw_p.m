figure(1); clf;
axis manual
axis equal;
ylim([-0.7 0.7]);
xlim([-1.0 1.0]);
h = hgtransform;
rectangle('Position',[-0.51,-0.03,0.06,0.06],'Curvature',[1,1],'FaceColor',[0.8,0.8,0.8],'Parent',h);
rectangle('Position',[0.45,-0.03,0.06,0.06],'Curvature',[1,1],'FaceColor',[0.8,0.8,0.8],'Parent',h);
h2 = rectangle('Position',[-0.48,-0.03,0.96,0.06],'FaceColor',[0.8,0.8,0.8],'Parent',h);
h2.LineStyle='none';
h2 = rectangle('Position',[-0.44,0.02,0.88,0.01],'FaceColor','white','Parent',h);
h2.LineStyle='none';
plot([-0.48,-0.44,-0.44,0.44,0.44,0.48],[0.03,0.03,0.02,0.02,0.03,0.03],'k','Parent',h);
plot([-0.48,0.48],[-0.03,-0.03],'k','Parent',h);
plot(-0.48,0,'k.','LineWidth',3,'Parent',h);
plot(0.48,0,'k.','LineWidth',3,'Parent',h);
h.Matrix = makehgtform('translate',[0 -0.04 0]);

h2 = hgtransform;
rectangle('Position',[-0.05,-0.02,0.1,0.04],'FaceColor',[0.8,0.8,0.8],'Parent',h2);

h3 = hgtransform('Parent',h2);
rectangle('Position',[-0.02,-0.15,0.04,0.17],'Curvature',[1,0.2],'FaceColor',[0.95,0.95,0.95],'Parent',h3);
plot(0,0,'k.','LineWidth',3,'Parent',h2);

h3b = hgtransform('Parent',h3);
h3b.Matrix = makehgtform('translate',[0 -0.15 0]);

h4 = hgtransform('Parent',h3b);
rectangle('Position',[-0.02,-0.294,0.04,0.314],'Curvature',[1,0.15],'FaceColor',[0.95,0.95,0.95],'Parent',h4);
plot(0,0,'k.','LineWidth',3,'Parent',h3b);

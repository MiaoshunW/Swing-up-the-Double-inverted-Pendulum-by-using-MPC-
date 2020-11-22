framerate = 50;

l = 0.5;
b = 0.02;
g = 9.81;
gl = g*l;
J_eff = (4.0*l*l + 0.02*0.02)/12.0;
c = 6/(4*l + b^2/l);
T1 = 0.025;

mpc = importfile5('mpc2.dat');
x = mpc(:,[1 3]);
alpha = mpc(:,[1 5]);
t = mpc(:,1);
I = find(diff(t)==0);
while numel(I)>0
    t(I) = t(I)-1e-12;
    I = find(diff(t)==0);
end
tmax = t(end);

u = mpc(:,[1 2]);
u(end,1) = round(u(end,1)*1000)/1000;
sim('test_nlin_interp2');


up_bnd = [0.5, 0.38, 0.5, 1e20, 25, 0.1];
lw_bnd = [-0.5, -0.38, -0.5, -1e20, -25, 0];
weight = [0.5, 0.0, 1.0, 0.0, 0.1, 1.0];

max_bnd = max(up_bnd(2:end),-lw_bnd(2:end));
max_bnd(3) = pi;
weightmax = sum(weight);
weight(1:5) = weight(1:5)./(max_bnd.^2)/weightmax;


%E2max = max_bnd(2)^2 + J_eff*max_bnd(4)^2;
%E2max = max( E2max, gl*2.0);
E2max = gl*2.5;
weight(6) = weight(6)/E2max^2/weightmax;

obj = cos(mpc(:,5)) - 1;
obj = obj .* (mpc(:,4).*mpc(:,6)*l+gl);
obj = obj + mpc(:,6).^2*J_eff;
obj = obj + mpc(:,4).^2;
obj = obj.^2 * weight(6);
Pendelenergie = obj;

dalpha = mod(alpha(:,2)+pi,2*pi)-pi;
Wagenlage = mpc(:,3).^2*weight(1);
obj = obj + Wagenlage;
obj = obj + mpc(:,4).^2*weight(2);
Pendellage = dalpha.^2*weight(3);
obj = obj + Pendellage;
obj = obj + mpc(:,6).^2*weight(4);
Stellenergie = mpc(:,7)*weight(5);
obj = obj + Stellenergie;
%Guetemass = mpc(:,8);

figure(2);
subplot(2,1,1)
plot(mpc(:,1),[obj,Pendelenergie,Wagenlage,Pendellage,Stellenergie]);%,Guetemass]);
legend('Gütemaß_lokal','Pendelenergie','Wagenlage','Pendellage','Stellenergie');%,'Gütemaß_aus_Programm');
subplot(2,1,2)
plot(tout,x);
legend('Wagenlage','Wagengeschwindigkeit','Pendellage','Pendelgeschwindigkeit');%,'Gütemaß_aus_Programm');


tmax = min(6.5,mpc(end,1));

dt = 1/framerate;
tv = 0:dt:tmax;
alpha1 = interp1(tout,-x(:,3)+pi,tv);
x1 = interp1(tout,x(:,1),tv);
alpha = interp1(t,-mpc(:,5)+pi,tv);
x = interp1(t,mpc(:,3),tv);

mpc = importfile6('mpc3.dat');
x_op = mpc(:,3);
alpha_op = -mpc(:,5)+pi;
t_op = mpc(:,1);
I_start = find(mpc(:,8)==1);
I_op_start = find(mpc(:,8)==2);

v = VideoWriter('generic','MPEG-4');
v.FrameRate = framerate;
open(v);

traj_counter = 1;
opt_counter = 1;

try
    draw_p;
%     h4 = hgtransform;
%     rectangle('Position',[-0.05,-0.02,0.1,0.04],'FaceColor',[0.8,0.8,0.8],'Parent',h4);
% 
%     h5 = hgtransform('Parent',h4);
%     rectangle('Position',[-0.01,-0.49,0.02,0.5],'FaceColor',[0.95,0.95,0.95],'Parent',h5);
%     plot(0,0,'k.','LineWidth',3,'Parent',h4);

    hold all;
    F = getframe();
    for i = 1:50
        writeVideo(v,F);
    end

    for j = 1:floor(tmax/dt)

        h2.Matrix = makehgtform('translate',[x1(j) 0 0]);
        h3.Matrix = makehgtform('zrotate',alpha1(j));
%         h4.Matrix = makehgtform('translate',[x(j) 0 0]);
%         h5.Matrix = makehgtform('zrotate',alpha(j));
        
        if tv(j) >= t_op(I_start(opt_counter))
            I = I_start(opt_counter):I_op_start(opt_counter)-1;
            x_traj = x_op(I) + 0.5*sin(alpha_op(I));
            y_traj = -0.5*cos(alpha_op(I));
            
            h(traj_counter) = plot(x_traj,y_traj,'Color',[1 0 0]);
            
            I = I_op_start(opt_counter):I_start(opt_counter+1)-1;
            x_traj = x_op(I) + 0.5*sin(alpha_op(I));
            y_traj = -0.5*cos(alpha_op(I));
            
            h(traj_counter+1) = plot(x_traj,y_traj,'Color',[0 1 0]);

            traj_counter = traj_counter + 2;
            opt_counter = opt_counter + 1;
        end
        
        i = 1;
        while traj_counter-2 >= 2*i && h(traj_counter-2*i-1).isvalid
            Color = h(traj_counter-2*i-1).Color;
            if Color(1) < 0.975
                h(traj_counter-2*i-1).Color = h(traj_counter-2*i-1).Color + [0.025 0 0.025];
                h(traj_counter-2*i-2).Color = h(traj_counter-2*i-2).Color + [0 0.025 0.025];
            else
                h(traj_counter-2*i-1).delete;
                h(traj_counter-2*i-2).delete;
            end
            i = i+1;
        end

        F = getframe();
        writeVideo(v,F);
        if opt_counter == 60
            break;
        end
    end
    close(v);
catch
   close(v);
   error(lasterr);
end

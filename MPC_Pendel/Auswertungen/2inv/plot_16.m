framerate = 50;

mpc = importfile5('mpc2.dat');
x = mpc(:,[1 3]);
alpha = mpc(:,[1 5]);
t = mpc(:,1);
I = find(diff(t)==0);
t(I) = t(I)-0.0001;
tmax = t(end);


up_bnd = [0.5, 0.38, 0.5, 1e20, 25, 0.1];
lw_bnd = [-0.5, -0.38, -0.5, -1e20, -25, 0];
weight = [0.5, 0.0, 1.0, 0.0, 0.02, 1.0];

max_bnd = max(up_bnd(2:end),-lw_bnd(2:end));
max_bnd(3) = pi;
weightmax = sum(weight);
weight(1:5) = weight(1:5)./(max_bnd.^2)/weightmax;

l = 0.5;
gl = 9.81*l;
J_eff = (4.0*l*l + 0.02*0.02)/12.0;

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

figure(2);
plot(mpc(:,1),[obj,Pendelenergie,Wagenlage,Pendellage,Stellenergie,mpc(:,8)]);
legend('Gütemaß','Pendelenergie','Wagenlage','Pendellage','Stellenergie','Entscheidung');

tmax = 6.5;

dt = 1/framerate;
tv = 0:dt:tmax;
alpha = interp1(t,-alpha(:,2)+pi,tv);
x = interp1(t,x(:,2),tv);

mpc = importfile3('mpc3.dat');
x_op = mpc(:,3);
alpha_op = -mpc(:,5)+pi;
t_op = mpc(:,1);
I_start = find(mpc(:,7)==0);
I_op_start = 1+find(diff(t_op)==0);

v = VideoWriter('generic','MPEG-4');
v.FrameRate = framerate*0.6; %40% langsamer sieht besser aus
open(v);

traj_counter = 1;
opt_counter = 1;

try
    draw_p; hold all;
    F = getframe();
    for i = 1:50
        writeVideo(v,F);
    end

    for j = 1:floor(tmax/dt)

        h2.Matrix = makehgtform('translate',[x(j) 0 0]);
        h3.Matrix = makehgtform('zrotate',alpha(j));
        
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
    end
    close(v);
catch
    close(v);
    error(lasterr);
end
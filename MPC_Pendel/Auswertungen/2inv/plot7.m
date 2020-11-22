framerate = 50;

mpc = importfile2('7.dat');
x = mpc(:,3);
alpha = -mpc(:,5)+pi;
t = mpc(:,1);
I = find(diff(t)==0);
t(I) = t(I)-0.0001;

%tmax = t(end);
tmax = 6;

dt = 1/framerate;
tv = 0:dt:tmax;
alpha = interp1(t,alpha,tv);
x = interp1(t,x,tv);

mpc = importfile3('7b.dat');
x_op = mpc(:,3);
alpha_op = -mpc(:,5)+pi;
t_op = mpc(:,1);
I_start = 1+[0; find(diff(t_op)<0)];
I_op_start = 1+find(diff(t_op)==0);

v = VideoWriter('7','MPEG-4');
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
%             i = 1;
%             while (traj_counter >= 2*i) && (i<4)
%                 h(traj_counter-2*i+1).Color = [1/4*i 1 1/4*i];
%                 h(traj_counter-2*i).Color = [1 1/4*i 1/4*i];
%                 i = i+1;
%             end
%             if traj_counter >= 2*i
%                 h(traj_counter-2*i+1).delete;
%                 h(traj_counter-2*i).delete;
%             end
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
framerate = 50;

mpc1 = importfile_neu_2('mpc2.dat');
x = mpc1(:,[1 7]);   % Wagenposition und Zeit
alpha = mpc1(:,[1 5]);   % Winkelposition 1 und Zeit
alpha2 = mpc1(:,[1 6]);   % Winkelposition 2 und Zeit
t = mpc1(:,1);   % Zeit
I = find(diff(t)==0);   % t ableiten, dann Stelle finden an der Ableitung 0 ist

while numel(I)>0
    t(I) = t(I)-1e-12;
    I = find(diff(t)==0);
end
tmax = t(end);

g = 9.81;		
T1 = 0.0395;	
a1 = 0.085;	
a2 = 0.157;	
l1 = 0.17;	
l2 = 0.314;	
m = 0.5;		
m1 = 0.162;	
m2 = 0.203;	
J1 = ((m1 * l1 * l1) / 12);	
J2 = ((m2 * l2 * l2) / 12);

up_bnd = [1.2, 25, 25, 1e20, 1e20, 0.38, 1.2, 0.5];
lw_bnd = [-1.2, -25, -25, -1e20, -1e20, -0.38, -1.2, 0];
weight = [0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.02, 1.0];

max_bnd = max(up_bnd(2:end),-lw_bnd(2:end));
max_bnd(3) = pi;    % zuvor 3? Fehler wegen c++ und matlab indizierung?
max_bnd(4) = pi;
weightmax = sum(weight);
weight(1:7) = weight(1:7)./(max_bnd.^2)/weightmax;

gl = 9.81 * l1;
J_eff = (m1 * l1 * l1) / 3;

E2max = m1*gl*2.5;
weight(8) = weight(8)/E2max^2/weightmax;


% obj = cos(mpc(:,5)) - 1;
% obj = obj .* (mpc(:,4).*mpc(:,6)*l+gl);
% obj = obj + mpc(:,6).^2*J_eff;
% obj = obj + mpc(:,4).^2;
% obj = obj.^2 * weight(6);
% Pendelenergie = obj;
% 
% dalpha = mod(alpha(:,2)+pi,2*pi)-pi;
% Wagenlage = mpc(:,3).^2*weight(1);
% obj = obj + Wagenlage;
% obj = obj + mpc(:,4).^2*weight(2);
% Pendellage = dalpha.^2*weight(3);
% obj = obj + Pendellage;
% obj = obj + mpc(:,6).^2*weight(4);
% Stellenergie = mpc(:,7)*weight(5);
% obj = obj + Stellenergie;
% %Guetemass = mpc(:,8);

% figure(2);
% subplot(2,1,1)
% plot(mpc(:,1),[obj,Pendelenergie,Wagenlage,Pendellage,Stellenergie]);%,Guetemass]);
% legend('Gütemaß_lokal','Pendelenergie','Wagenlage','Pendellage','Stellenergie');%,'Gütemaß_aus_Programm');
% subplot(2,1,2)
% plot(mpc(:,1),mpc(:,8));


tmax = min(20,mpc1(end,1));

dt = 1/framerate;
tv = 0:dt:tmax;
alpha = interp1(t,-alpha(:,2)+pi,tv);
x = interp1(t,x(:,2),tv);

mpc2 = importfile_neu_3('mpc3.dat');
x_op = mpc2(:,7);
alpha_op = -mpc2(:,5)+pi;
alpha_op2 = -mpc2(:,6)+pi;
t_op = mpc2(:,1);
I_start = find(mpc2(:,8)==1);
I_op_start = find(mpc2(:,8)==2);

v = VideoWriter('generic','MPEG-4');
v.FrameRate = framerate*0.4; %40% langsamer sieht besser aus
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
        h4.Matrix = makehgtform('zrotate',alpha2(j));
        
%         if tv(j) >= t_op(I_start(opt_counter))
%             I = I_start(opt_counter):I_op_start(opt_counter)-1;
%             x_traj = x_op(I) + 0.5*sin(alpha_op(I));
%             y_traj = -0.5*cos(alpha_op(I));
%             
%             h(traj_counter) = plot(x_traj,y_traj,'Color',[1 0 0]);
%             
%             I = I_op_start(opt_counter):I_start(opt_counter+1)-1;
%             x_traj = x_op(I) + 0.5*sin(alpha_op(I));
%             y_traj = -0.5*cos(alpha_op(I));
%             
%             h(traj_counter+1) = plot(x_traj,y_traj,'Color',[0 1 0]);
% 
%             traj_counter = traj_counter + 2;
%             opt_counter = opt_counter + 1;
%         end
%         
%         i = 1;
%         while traj_counter-2 >= 2*i && h(traj_counter-2*i-1).isvalid
%             Color = h(traj_counter-2*i-1).Color;
%             if Color(1) < 0.975
%                 h(traj_counter-2*i-1).Color = h(traj_counter-2*i-1).Color + [0.025 0 0.025];
%                 h(traj_counter-2*i-2).Color = h(traj_counter-2*i-2).Color + [0 0.025 0.025];
%             else
%                 h(traj_counter-2*i-1).delete;
%                 h(traj_counter-2*i-2).delete;
%             end
%             i = i+1;
%         end

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
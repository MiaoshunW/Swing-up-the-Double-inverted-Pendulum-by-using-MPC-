%% initialisation
framerate = 50;
mpc = importfile('Shun1.dat');

%% showing the input, states and deduced values like input "power" or pendulum energy over time
figure(3);
subplot(4,2,1)
plot(mpc(:,1),mpc(:,2));
legend('control value');
subplot(4,2,2)
plot(mpc(:,1),mpc(:,[3 4]));
legend('p1 velocity [rad/s]','p2 velocity [rad/s]');
subplot(4,2,3)
plot(mpc(:,1),mpc(:,[5 6]));
legend('p1 position [rad]','p2 position [rad]');
subplot(4,2,4)
plot(mpc(:,1),mpc(:,8));
legend('cart velocity [m/s]');
subplot(4,2,5)
plot(mpc(:,1),mpc(:,7));
legend('cart position [m]');
subplot(4,2,6)
plot(mpc(:,1),mpc(:,9));
legend('intput energy [J]');
mtit('input, states, deduced values');

%% showing the objective, both total and partial, over time
ziel   = [        0,    0,          0,      pi,       0,        0,        0            ];
weight = [        0,    0,         1.0,     1.0,     0.1,       0,        0.3,        1];
%        velocity 1,  velocity 2, angle 1, angle 2, cart pos, cart speed, input power, energy p1
up_bnd = [ 1.2,  25,   25,        1e20,    1e20,     0.38,       1.2,       0.5];
lw_bnd = [-1.2, -25,  -25,       -1e20,   -1e20,    -0.38,      -1.2,         0];
% bounds have input bound at first index, and no bound for energy p1 => ok!

m1 = 0.162;	% Mass pendulum 1 [kg]
m2 = 0.203;
a2 = 0.157;	
a1 = 0.085;	% Half pendulum neck 1 [m]
l1 = 0.17;  % pendulum neck 1 [m]
g  = 9.81;	% gravitation [m/s^2]
gl = 9.81 * l1;
J_eff = (m1 * l1 * l1) / 3; %effective maximum moment of inertia pendulum 1 [kg m^2]

max_bnd = max(up_bnd(2:end),-lw_bnd(2:end));
max_bnd(3:4) = pi;
weightmax = sum(weight);
weight(1:6) = weight(1:6)./(max_bnd(1:6).^2) / weightmax;
weight(7) = weight(7) / max_bnd(7) / weightmax;
weight(8) = weight(8) / (m1*g*a1+m2*g*l1+m2*g*a2)^2 / weightmax;

tmp = mpc(:,8) - ziel(6);
obj = tmp.*tmp * weight(6); %cart speed -> 0 ?

tmp = abs( mod(mpc(:,6) - ziel(4) + pi, pi*2) - pi);
Pendellage2 = tmp.*tmp * weight(4); %angle 2 -> wrap to pi missing! 
obj = obj + Pendellage2; 

tmp = abs(mod(mpc(:,5) - ziel(3) + pi, pi*2) - pi); % angle 1, wrapped to pi
Pendellage1=tmp.*tmp * weight(3); % standardization (0 top, 1 hanging down)
obj=obj+ Pendellage1;

tmp = mpc(:,3) - ziel(1);
obj = obj + tmp.*tmp * weight(1);    %velocity 1 -> 0 ?
tmp = mpc(:,4) - ziel(2);
obj = obj + tmp.*tmp * weight(2);     %velocity 2 

Stellenergie = (mpc(:,9)-ziel(7) )* weight(7); %input power
obj = obj + Stellenergie;

tmp = cos(mpc(:,5)) - cos(ziel(3));
tmp = tmp .*(l1*m2*g + g*m1*a1);
tmp = tmp + (cos(mpc(:,6)) - cos(ziel(4)))*a2*m2*g;
Pendelenergie = tmp.*tmp * weight(8);                     %angle 1 energy, most likely wrong !!!
obj = obj + Pendelenergie;


tmp = mpc(:,7) - ziel(5);
Wagenlage = tmp.*tmp * weight(5);                          %carriage position
obj = obj + Wagenlage;

figure(2);
plot(mpc(:,1),[obj,Pendelenergie,Wagenlage,Pendellage1,Pendellage2,Stellenergie]);%,Guetemass]);
legend('Total energy','pendulums energy','cart position','pendulum 1 position','pendulum 2 position','input power');%,'Gütemaß_aus_Programm');
title('Total and partial objective');

%% showing and saving the animation
x = mpc(:,[1 7]);
alpha = mpc(:,[1 5]);
alpha2 = mpc(:,[1 6]);

t = mpc(:,1);
I = find(diff(t)==0);
while numel(I)>0
    t(I) = t(I)-1e-12;
    I = find(diff(t)==0);
end

%tmax = min(6.5,mpc(end,1));
tmax = mpc(end,1);

dt = 1/framerate;
tv = 0:dt:tmax;
alpha =  interp1(t,-alpha(:,2)+pi,tv);
alpha2 = interp1(t,-alpha2(:,2)+pi,tv);
x = interp1(t,x(:,2),tv);

mpc = importfile1('Shun1b.dat');
x_op = mpc(:,7);
alpha_op  = -mpc(:,5)+pi;
alpha2_op = -mpc(:,6)+pi;
t_op = mpc(:,1);
I_start = find(mpc(:,10)==1);
I_op_start = find(mpc(:,10)==2);

v = VideoWriter('Shun1','MPEG-4');
v.FrameRate = framerate*0.6; %40% langsamer sieht besser aus
open(v);

traj_counter = 1;
opt_counter = 1;
opt_c_max = min(numel(I_op_start),numel(I_start)-1);

try
    draw_p; hold all;
    F = getframe();
    for i = 1:50
        writeVideo(v,F);
    end

    for j = 1:floor(tmax/dt)

        h2.Matrix = makehgtform('translate',[x(j) 0 0]);
        h3.Matrix = makehgtform('zrotate',alpha(j));
        h4.Matrix = makehgtform('zrotate',alpha2(j)+alpha(j));
        
        if tv(j) >= t_op(I_start(opt_counter))
            I = I_start(opt_counter):I_op_start(opt_counter)-1;
            x_traj = x_op(I) + 0.5*sin(alpha_op(I));
            y_traj = -0.5*cos(alpha_op(I));
            
            %h(traj_counter) = plot(x_traj,y_traj,'Color',[1 0 0]);
            
            I = I_op_start(opt_counter):I_start(opt_counter+1)-1;
            x_traj = x_op(I) + 0.5*sin(alpha_op(I));
            y_traj = -0.5*cos(alpha_op(I));
            
            %h(traj_counter+1) = plot(x_traj,y_traj,'Color',[0 1 0]);

            traj_counter = traj_counter + 2;
            opt_counter = opt_counter + 1;
        end
        
        %{
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
        %}
        
        F = getframe();
        writeVideo(v,F);
        if opt_counter == opt_c_max
            break;
        end
    end
    close(v);
%%{
catch
   close(v);
   error(lasterr);
end
%}

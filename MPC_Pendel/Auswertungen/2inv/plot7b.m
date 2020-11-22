mpc = importfile3('7.dat');
x = mpc(:,[1 3]);
alpha = mpc(:,[1 5]);
%tmax = mpc(end,1);
tmax = 7;

up_bnd = [0.5, 0.38, 0.5, 1e20, 25, 0.1];
lw_bnd = [-0.5, -0.38, -0.5, -1e20, -25, 0];
weight = [0.2, 0.0, 1.0, 0.0, 0.1, 1.0];

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

plot(mpc(:,1),[obj,Pendelenergie,Wagenlage,Pendellage,Stellenergie]);
legend('Gütemaß','Pendelenergie','Wagenlage','Pendellage','Stellenergie');
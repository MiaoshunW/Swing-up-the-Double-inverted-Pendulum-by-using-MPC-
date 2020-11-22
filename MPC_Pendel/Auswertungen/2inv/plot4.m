mpc = importfile('4.dat');
x = mpc(:,[1 2]);
alpha = mpc(:,[1 4]);
tmax = mpc(end,1);

up_bnd = [0.5, 0.38, 0.5, 1e20, 25, 100];
lw_bnd = [-0.5, -0.38, -0.5, -1e20, -25, 0];
weight = [0.01, 0.1, 1, 0.1, 0.01, 1];

max_bnd = max(up_bnd(2:end),-lw_bnd(2:end));
max_bnd(3) = pi;
weightmax = sum(weight);
weight(1:5) = weight(1:5)./(max_bnd.^2)/weightmax;

E2max = max_bnd(2)^2 + J_eff*max_bnd(4)^2;
E2max = max( E2max, gl*2.0);
weight(6) = weight(6)/E2max^2/weightmax;

obj = cos(mpc(:,4)) - 1;
obj = obj .* (mpc(:,3).*mpc(:,5)*l+gl);
obj = obj + mpc(:,5).^2*J_eff;
obj = obj + mpc(:,3).^2;
obj = obj.^2 * weight(6);

dalpha = mod(alpha(:,2)+pi,2*pi)-pi;
obj = obj + mpc(:,2).^2*weight(1);
obj = obj + mpc(:,3).^2*weight(2);
obj = obj + dalpha.^2*weight(3);
obj = obj + mpc(:,5).^2*weight(4);

plot(mpc(:,1),obj)
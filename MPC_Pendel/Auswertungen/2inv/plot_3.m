mpc = importfile('3.dat');
x = mpc(:,[1 2]);
alpha = mpc(:,[1 4]);
tmax = mpc(end,1);

up_bnd = [0.5, 0.38, 0.5, 1e20, 25, 100];
lw_bnd = [-0.5, -0.38, -0.5, -1e20, -25, -1000];
max_bnd = max(up_bnd,-lw_bnd);

E2max = max_bnd(3)^2 + J_eff*max_bnd(5)^2;
E2max = max( E2max, gl*2.0);
E2max = 1 / E2max / E2max;

obj = cos(mpc(:,4)) - 1;
obj = obj .* (mpc(:,3).*mpc(:,5)*l+gl);
obj = obj + mpc(:,5).^2*J_eff;
obj = obj + mpc(:,3).^2;
obj = obj.^2 * E2max;

plot(mpc(:,1),obj)
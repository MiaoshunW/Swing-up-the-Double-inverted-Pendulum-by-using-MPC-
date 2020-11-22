mpc = importfile('1.dat');
x = mpc(:,[1 2]);
alpha = mpc(:,[1 4]);
tmax = mpc(end,1);

gl3 = 14.715;
l2b2 = 0.2504;
E2max = 0.00065326787;

obj = mpc(:,5).^2*l2b2/4.0;
obj = obj + gl3*(cos(mpc(:,4)) - 1);
obj = obj.^2 * E2max;

plot(mpc(:,1),obj)
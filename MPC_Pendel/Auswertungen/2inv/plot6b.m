framerate = 50;

mpc = importfile2('6.dat');
x = mpc(:,3);
alpha = -mpc(:,5)+pi;
t = mpc(:,1);
I = find(diff(t)==0);
t(I) = t(I)-0.0001;

%tmax = t(end);
tmax = 7;

dt = 1/framerate;
tv = 0:dt:tmax;
alpha = interp1(t,alpha,tv);
x = interp1(t,x,tv);

v = VideoWriter('6','MPEG-4');
v.FrameRate = framerate*0.8; %20% langsamer sieht besser aus
open(v);

try
    draw_p;
    for j = 1:floor(tmax/dt)

        h2.Matrix = makehgtform('translate',[x(j) 0 0]);
        h3.Matrix = makehgtform('zrotate',alpha(j));

        F = getframe();
        writeVideo(v,F);
    end
    close(v);
catch
    close(v);
    error(lasterr);
end
musigma = importfile4('mu_sigma.dat', 7, 166);

mu = musigma(:,1:2:end);
sigma = musigma(:,2:2:end);

mu = reshape(mu,[8,5,4,5]);
mu = permute(mu,[3 2 1 4]);

sigma = reshape(sigma,[8,5,4,5]);
sigma = permute(sigma,[3 2 1 4]);

x = 0:0.1:0.3;
x_pkt = -0.5:0.25:0.5;
alpha = -pi:pi/4:pi-pi/4;
alpha_pkt = -2:1:2;

figure(1)
for i = 1:4
    for j = 1:5
        subplot(4,5,j+5*i-5);
        mesh(x_pkt,alpha,squeeze(mu(i,:,:,j))');
        title([' x = ',num2str(x(i)),', \alpha_{pkt} = ',num2str(alpha_pkt(j))])
        zlim([0 0.15])
    end
end

figure(2)
for i = 1:4
    for j = 1:5
        subplot(4,5,j+5*i-5);
        mesh(x_pkt,alpha,squeeze(sigma(i,:,:,j))');
        title([' x = ',num2str(x(i)),', \alpha_{pkt} = ',num2str(alpha_pkt(j))])
        zlim([0 0.04])
    end
end

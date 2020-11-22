pv = 3.8:0.05:4.2;
figure(1);clf;
hold all;
for i = 1:21
  p = pv(i);
  sim('test');
  plot(E(:,2));
end
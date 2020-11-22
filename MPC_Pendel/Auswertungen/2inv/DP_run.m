cla
h = DoublePendulum();
for a=pi/5:.1:2*pi/5
  h.setAngles(a,-a/2);
end

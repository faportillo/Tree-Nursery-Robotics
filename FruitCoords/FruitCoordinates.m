R1 = se2(5,4,45*pi/180);
R2 = se2(2, -0.2, -30*pi/180);
C1 = R1*R2;
trplot2(R1, 'frame', 'R', 'color', 'b')
hold on
trplot2(C1, 'frame', 'C', 'color', 'r')
axis([0 15 -2 7])
xticks(0:2:14)
numFruit = 10;
fruit = [-1.7326, 4.6739, -1.8295, -1.6129, 3.0871, -1.4017, 6.0614, 6.1569, 4.9803, -0.9346 ; -2.7593, -2.8955, -2.4566, -2.7695, -2.6233, -2.9581, -2.9787, -2.5992, -3.3158, -3.2678];

fprintf('World Coordinates: ')
for i = 1:numFruit
  f = fruit(:, i);
  worldFruit = homtrans(C1, f);
  fprintf('(%0.3f, %0.3f) ', worldFruit(1), worldFruit(2))
  plot_point(worldFruit, 'go')
end
fprintf('\n')
%{
 RunNurseryAnalysis.m
 1 June 2018
 Main file to run to perform analysis of nursery
%}

addpath '../Common'
clear
close all

PLOT_REAL_TREES = true;
start = [5,5];

global bitmap;
[K, x_im, y_im] = generateNursery();


if PLOT_REAL_TREES
  figure
  imagesc(x_im, y_im, flipud(bitmap)); %imagesc flips the bitmap rows, so correct this
  set(gca,'YDir','normal');
end
RC = struct('R', 500, 'C', 500);



RL = 20;      %meters
Gmax = 60 * pi/180;
L = 3;
X = 1;
Y = 2;
Vm = 4;

initAlloc = 1000;
endI = 2 * K + 2;
C = struct('W', 2.5,...   %row width [m]
  'L', L,...              %wheelbase [m]
  'Rw', 0.5,...           %radius [m]
  'Vmax', Vm,...          %v max     [m/s]
  'Gmax', Gmax,...        %gamma max [radians]
  'Ld',   2.0,...         %min distance to first navigatable point [meters]
  'dt',   0.001,...       %seconds
  'DT',   0.01,...        %seconds
  'T',    600.0,...       %total move to point time allowed
  'RL', 20, ...           %
  'HUGE', 10^9,...        %
  'rowLength', 20, ...    %length of row [m]
  'rowWidth', 3, ...      %center-to-center row width [m]
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'endI', endI,...        %number of nodes
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

[nodes, DMAT] = Nursery.MakeNodes(K);
robot = DrawableRobot(start(X), start(Y), pi/2, C.Rw, C.L, C.Gmax, C.Vmax, ...
  C.Ld);
nursery = Nursery(bitmap, K, C.rowLength, C. rowWidth, nodes, robot);

%test of my functions    -Tim
diam = 0.3 .* rand(20 * K, 1) + 0.2;
for i = 1:(20*K)
  nursery.AddTree(i, i*2, diam(i), floor((i-1) / 20) + 1)
end
nursery.WriteResults('testResults.txt');

fprintf('Done')

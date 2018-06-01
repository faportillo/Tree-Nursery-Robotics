%{
 RunNurseryAnalysis.m
 1 June 2018
 Main file to run to perform analysis of nursery
%}

addpath '../Common'
clear
close all

RC = struct('R', 500, 'C', 500);

[K, map] = Nursery.InterpretInput('filename', RC);



RL = 20;      %meters
Gmax = 60 * pi/180;
L = 3;
X = 1;
Y = 2;
N = 10;
Vm = 4;

initAlloc = 1000;
endI = 2 * N + 2;
C = struct('W', 2.5,...   %row width [m]
  'L', L,...              %wheelbase [m]
  'Rw', 0.5,...           %radius [m]
  'Vmax', Vm,...          %v max     [m/s]
  'Gmax', Gmax,...        %gamma max [radians]
  'Ld',   2.0,...         %min distance to first navigatable point [meters]
  'dt',   0.001,...       %seconds
  'DT',   0.01,...        %seconds
  'T',    600.0,...       %total move to point time allowed
  'N', N,...              %crop rows
  'RL', 20, ...           %
  'HUGE', 10^9,...        %
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'endI', endI,...        %number of nodes
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );
%{
path = zeros(2, 3);
planner = PathPlanner(path, C.Rmin, C.W);
xs = (C.W / 2):C.W:(C.W / 2 + C.W * (C.N - 1));
robot = DrawableRobot(-4 * C.W, RL / 2, pi/2, C.Rw, C.L, C.Gmax, C.Vmax, C.Ld);
nodes = [robot.x, xs, xs, robot.x;
  robot.y, zeros(1, C.N), (zeros(1, C.N) + RL), robot.y];
%}
  
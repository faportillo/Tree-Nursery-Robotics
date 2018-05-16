%{
 Lidar2D.m
 Tim Ambrose
 9 May 2018
 
%}
addpath '../Common'
clear
close all

RL = 20;      %meters
Gmax = 60 * pi/180;
L = 3;
X = 1;
Y = 2;
N = 10;
Vm = 4;

initAlloc = 1000;

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
  'R', 500,...            %rows
  'C', 500,...            %columns
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'endI', endI,...        %number of nodes
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

grid = ones(R, C);







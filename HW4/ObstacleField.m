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
  'Fcr', 1.0,...          %repelling force constant
  'Fct', 1.0,...          %attracting force constant
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

map=zeros(C.R, C.C);
Xmax = 150; Ymax = 150;
[oddsGrid, occGrid] = getObstacleField(map, C.R, C.C, Xmax, Ymax);
%{
imagesc(occGrid)
colorbar
figure
imagesc(oddsGrid)
colorbar
%}
Fr = zeros(C.C, C.R, 2);   % (x, y, {x, y})  third dim is vector xy components
Ft = zeros(C.C, C.R, 2);   %attracting forces
for y = 1:C.R
  for x = 1:C.C
    d = sqrt((x - robot.x)^2 + (y - robot.y)^2);
    %[(x - robot.x);(y - robot.y)];
    Ft(x, y, :) = C.Fct * [((x - robot.x) / d), ((y - robot.y) / d)];
    Fr(x, y, :) = Ft(x, y, :) ./ C.Fct .* C.Fcr .* occGrid(y, x) ./ (d^2);
  end
end




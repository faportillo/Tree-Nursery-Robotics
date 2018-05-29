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

C = struct('L', L,...     %wheelbase [m]
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
  'd0', 2.0,...           %threshold distance for repulsion
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 200, ...   %max range of laser
  'angleSpan', pi, ...    %angle span of laser sweep
  'angleStep', pi/360, ...   %step of laser sweep
  'Rmin', L / tan(Gmax), ... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

map=zeros(C.R, C.C);
Xsw=20; Ysw = 30;
Xne=Xsw + 40; Yne= Ysw + 20;
xMax = 100; yMax = 100;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, xMax, yMax, C.R, C.C);
[Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, C.R, C.C);
map(Ine:Isw, Jsw:Jne) = 1;

robot = DrawableRobot(10, 10, pi/2, C.Rw, C.L, C.Gmax, C.Vmax, C.Ld);
frame = [0, xMax, 0, yMax];
robot.DrawRobot(frame, true);

hold on
[~, occGrid] = getObstacleField(map, C, xMax, yMax, robot);
%{
[ii,jj] = find(occGrid);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, C.R, C.C);
plot(xx, yy, 'b.')
%
figure
imagesc(occGrid)
colorbar
%}

Fr = zeros(C.C, C.R, 2);   % (x, y, {x, y})  third dim is vector xy components
Ft = zeros(C.C, C.R, 2);   %attracting forces
for y = 1:C.R
  for x = 1:C.C
    d = sqrt((x - robot.x)^2 + (y - robot.y)^2);
    %[(x - robot.x);(y - robot.y)];
    Fr(x, y, :) = Ft(x, y, :) ./ C.Fct .* C.Fcr .* occGrid(y, x) ./ (d^2);
  end
end

pathPoint = 'b.';
tauG = 0.15;
tauV = 0.5;
s = 0;
skidDR = 0;
skidDF = 0;
vd = C.Vmax;

for t = 0:C.DT:(C.T - C.DT)
  redraw = mod(t, C.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end

  if k > length(stepErr)
    stepErr = [stepErr, zeros(1, length(stepErr))];
  end
  [gammaD, err, prev, errX, errY] = planner1.FirstFeasiblePoint(robot, prev);
  delete(lookAhead)
  lookAhead = plot(pathPoints(X, prev), pathPoints(Y, prev), 'k*');
  stepErr(k) = err;
  k = k + 1;

  robot.MoveEulerAck_RedrawRobot(gammaD, vd, C, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if prev == planner1.nPoints && abs(errX) + abs(errY) < C.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end



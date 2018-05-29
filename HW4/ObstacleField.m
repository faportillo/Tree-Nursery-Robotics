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
goalX = 60;
goalY = 80;
N = 10;
Vm = 4;
span = pi;
C = 500; R = 500;
xMax = 100; yMax = 100;

initAlloc = 1000;

CC = struct('L', L,...     %wheelbase [m]
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
  'R', R,...            %rows
  'C', C,...            %columns
  'obstWindow', 40, ...   %window to calculte repulsive forces (meters)
  'Fcr', 0.0,...         %repelling force constant
  'Fct', 10.0,...         %attracting force constant
  'd0', 2.0,...           %threshold distance for repulsion
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 200, ...   %max range of laser
  'angleSpan', span, ...  %angle span of laser sweep
  'angleStep', span/360, ... %step of laser sweep
  'Rmin', L / tan(Gmax), ... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.05 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

map=zeros(R, C);
Xsw=20; Ysw = 30;
Xne=Xsw + 40; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, xMax, yMax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

robot = DrawableRobot(80, 40, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
frame = [0, xMax, 0, yMax];
robot.DrawRobot(frame, true);


hold on
%{
figure
imagesc(occGrid)
colorbar
%}

% (x, y, [x y])  third dimension is vector xy components
FrGrid = zeros(2 * CC.obstWindow + 1, 2 * CC.obstWindow + 1, 2);

pathPoint = 'b.';
tauG = 0.15;
tauV = 0.5;
s = 0;
skidDR = 0;
skidDF = 0;
vd = CC.Vmax;
obstPlot = plot(0,0,'k.');
occGrid = getObstacleField(map, CC, xMax, yMax, robot);
fcr = CC.Fcr;

for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  
  if redraw
    occGrid = getObstacleField(map, CC, xMax, yMax, robot);
    [ii,jj] = find(occGrid);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'b.');
  end
  
  rx = robot.x;
  ry = robot.y;
  errX = goalX - rx;
  errY = goalY - ry;
  Ft = CC.Fct / sqrt(errX^2 + errY^2) .* [errX, errY];
  
  xL = round(max(1, rx - CC.obstWindow));
  xR = round(min(xMax, rx + CC.obstWindow));
  yD = round(max(1, ry - CC.obstWindow));
  yU = round(min(yMax, ry + CC.obstWindow));
  FrX = 0;
  FrY = 0;
  parfor x = xL:xR
    J = round((x / xMax) * (C-1)) + 1;
    for y = yD:yU
      I = round(((yMax - y) / yMax) * (R-1)) + 1;
      d = max(sqrt((x - rx)^2 + (y - ry)^2), 1e-6);
      occ = occGrid(I, J) * fcr / (d^3);
      FrX = FrX + occ * (x - rx);
      FrY = FrY + occ * (y - ry);
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  gammaD = atan2(Ry, Rx);
  %vd = sqrt(Rx^2 + Ry^2);
  
  robot.MoveEulerAck_RedrawRobot(gammaD, vd, CC, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if abs(errX) + abs(errY) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end



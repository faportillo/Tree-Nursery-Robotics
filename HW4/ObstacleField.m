%{
 ObstacleField.m
 Tim Ambrose
 30 May 2018
 
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
occThresh = 0.5;

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
  'R', R,...              %rows
  'C', C,...              %columns
  'obstWindow', 10, ...   %window to calculte repulsive forces (meters)
  'Fcr', 20.0,...         %repelling force constant
  'Fct', 1.0,...          %attracting force constant
  'd0', 2.0,...           %threshold distance for repulsion
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 200, ...   %max range of laser
  'angleSpan', span, ...  %angle span of laser sweep
  'angleStep', span/360, ... %step of laser sweep
  'Rmin', L / tan(Gmax), ... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.05 / Vm,... %# of DT to redraw robot for pursuit controller
  'rescan', 10, ...       %rescan every X redraws
  'aniPause', 0.001 ...   %animation pause
  );

map=zeros(R, C);
Xsw=20; Ysw = 30;
Xne=Xsw + 40; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, xMax, yMax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;

robot = DrawableRobot(10, 10, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
frame = [0, xMax, 0, yMax];
robot.DrawRobot(frame);


hold on
[ii,jj] = find(map > occThresh);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
obstclesPlot = plot(xx, yy, 'r.');

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
probGrid = zeros(R, C) + 0.1;
[probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, robot);
fcr = CC.Fcr;

scan = CC.redrawT * CC.rescan;
robotOr = pi/2;

%-------------------------PART 2------------------------------------
for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  if mod(t, scan) == 0
    [probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, ...
      robot);
  end
  if redraw
    [ii,jj] = find(probGrid > occThresh);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'k.');
  end
  
  rx = robot.x;
  ry = robot.y;
  rt = robot.theta;
  g = robot.RobotPoint([goalX; goalY]);
  %errX = g(X) - rx;
  %errY = g(Y) - ry;
  Ft = CC.Fct / sqrt(g(X)^2 + g(Y)^2) .* [g(X), g(Y)];
  
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
      p = probGrid(I, J);
      if p > occThresh
        rp = h2e(se2(rx, ry, rt) \ e2h([x;y]));
        d = max(sqrt(rp(X)^2 + rp(Y)^2), 1e-6);
        occ = p * fcr / (d^3);
        FrX = FrX + occ * rp(X);
        FrY = FrY + occ * rp(Y);
      end
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  gammaD = atan2(Rx, -Ry) + rt;
  %vd = sqrt(Rx^2 + Ry^2);
  
  robot.MoveEulerAck_RedrawRobot(gammaD, vd, CC, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if abs(g(X)) + abs(g(Y)) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end


%--------------------------------PART 3---------------------------------
CC.L = 2.5;
CC.Gmax = 45 * pi/180;
vd = 1;
CC.Vmax = vd;
xMax = 50; yMax = 50;
robot = DrawableRobot(25, 0, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
robot.dim = [2.5, 1.5];
goalX = 25; goalY = 30;

figure
frame = [0, xMax, 0, yMax];
robot.DrawRobot(frame);
hold on
map=zeros(R, C);
Xsw=25; Ysw = 15;
Xne=Xsw + 1; Yne= Ysw + 1;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, xMax, yMax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;
[ii,jj] = find(map > occThresh);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
obstclesPlot = plot(xx, yy, 'r.');

%---------------------------PART 3 Skid Steering--------------------------

for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  if mod(t, scan) == 0
    [probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, ...
      robot);
  end
  if redraw
    [ii,jj] = find(probGrid > occThresh);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'k.');
  end
  
  rx = robot.x;
  ry = robot.y;
  rt = robot.theta;
  g = robot.RobotPoint([goalX; goalY]);
  Ft = CC.Fct / sqrt(g(X)^2 + g(Y)^2) .* [g(X), g(Y)];
  
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
      p = probGrid(I, J);
      if p > occThresh
        rp = h2e(se2(rx, ry, rt) \ e2h([x;y]));
        d = max(sqrt(rp(X)^2 + rp(Y)^2), 1e-6);
        occ = p * fcr / (d^3);
        FrX = FrX + occ * rp(X);
        FrY = FrY + occ * rp(Y);
      end
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  gammaD = min(max(atan2(Rx, -Ry) - robotOr, -CC.Gmax), CC.Gmax);
  if gammaD > 0
    Wl = vd / CC.Rw;
    Wr = Wl * (1 - gammaD / CC.Gmax);
  else
    Wr = vd / CC.Rw;
    Wl = Wr * (1 - abs(gammaD) / CC.Gmax);
  end
  
  for tt = 0:CC.dt:(CC.DT - CC.dt)
    robot.MoveEulerDiff_MaybeRedraw(Wl, Wr, CC, 0, 0, 0, frame, ...
      pathPoint, prevPos, redraw);
  end
  if abs(g(X)) + abs(g(Y)) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end


%------------------------PART 3 Ackerman-------------------------
figure
frame = [0, xMax, 0, yMax];
robot = DrawableRobot(25, 0, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
robot.DrawRobot(frame);
hold on
[ii,jj] = find(map > occThresh);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
obstclesPlot = plot(xx, yy, 'r.');

scan = scan * 10;
for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  if mod(t, scan) == 0
    [probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, ...
      robot);
  end
  if redraw
    [ii,jj] = find(probGrid > occThresh);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'k.');
  end
  
  rx = robot.x;
  ry = robot.y;
  rt = robot.theta;
  g = robot.RobotPoint([goalX; goalY]);
  Ft = CC.Fct / sqrt(g(X)^2 + g(Y)^2) .* [g(X), g(Y)];
  
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
      p = probGrid(I, J);
      if p > occThresh
        rp = h2e(se2(rx, ry, rt) \ e2h([x;y]));
        d = max(sqrt(rp(X)^2 + rp(Y)^2), 1e-6);
        occ = p * fcr / (d^3);
        FrX = FrX + occ * rp(X);
        FrY = FrY + occ * rp(Y);
      end
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  gammaD = atan2(Rx, -Ry) + rt;
  
  robot.MoveEulerAck_RedrawRobot(gammaD, vd, CC, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if abs(g(X)) + abs(g(Y)) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end



%--------------------------------PART 4---------------------------------
CC.L = 2.5;
CC.obstWindow = 1;
fcr = 20;
CC.Gmax = 45 * pi/180;
vd = 1;
CC.Vmax = vd;
xMax = 50; yMax = 50;
figure
robot = DrawableRobot(25, 0, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
robot.dim = [2.5, 1.5];
goalX = 25; goalY = 30;
obstPlot = plot(0,0,'k.');

frame = [0, xMax, 0, yMax];
robot.DrawRobot(frame);
hold on
map=zeros(R, C);
Xsw=25; Ysw = 15;
for i = 1:4
  Xne=Xsw + 0.3; Yne= Ysw + 0.3;
  [Isw, Jsw] = XYtoIJ(Xsw, Ysw, xMax, yMax, R, C);
  [Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, R, C);
  map(Ine:Isw, Jsw:Jne) = 1;
  
  Xsw2 = Xsw - 3.3; Ysw2 = Ysw - 0.3;
  Xne=Xsw2 + 0.3; Yne= Ysw2 + 0.3;
  [Isw, Jsw] = XYtoIJ(Xsw2, Ysw2, xMax, yMax, R, C);
  [Ine, Jne] = XYtoIJ(Xne, Yne, xMax, yMax, R, C);
  map(Ine:Isw, Jsw:Jne) = 1;
  
  Xsw = Xsw + 0.3;
  Ysw = Ysw - 1;
end
[ii,jj] = find(map > occThresh);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
obstclesPlot = plot(xx, yy, 'r.');
gammaD = 0;

%---------------------------PART 4 Skid Steering--------------------------
%scan = scan / 10;
rc = 0.35;
rcT = 0.1;
for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  if mod(t, scan) == 0
    [probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, ...
      robot);
  end
  if redraw
    [ii,jj] = find(probGrid > occThresh);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'k.');
  end
  
  rx = robot.x;
  ry = robot.y;
  rt = robot.theta;
  g = robot.RobotPoint([goalX; goalY]);
  Ft = CC.Fct / sqrt(g(X)^2 + g(Y)^2) .* [g(X), g(Y)];
  
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
      p = probGrid(I, J);
      if p > occThresh
        rp = h2e(se2(rx, ry, rt) \ e2h([x;y]));
        d = max(sqrt(rp(X)^2 + rp(Y)^2), 1e-6);
        occ = p * fcr / (d^3);
        FrX = FrX + occ * rp(X);
        FrY = FrY + occ * rp(Y);
      end
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  newG = (rcT * (atan2(Rx, -Ry) - robotOr) + (rc - rcT) * gammaD) / rc;
  gammaD = min(max(newG, -CC.Gmax), CC.Gmax);
  if gammaD > 0
    Wl = vd / CC.Rw;
    Wr = Wl * (1 - abs(gammaD) / CC.Gmax);
  else
    Wr = vd / CC.Rw;
    Wl = Wr * (1 - abs(gammaD) / CC.Gmax);
  end
  
  for tt = 0:CC.dt:(CC.DT - CC.dt)
    robot.MoveEulerDiff_MaybeRedraw(Wl, Wr, CC, 0, 0, 0, frame, ...
      pathPoint, prevPos, redraw);
  end
  if abs(g(X)) + abs(g(Y)) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end


%------------------------PART 4 Ackerman-------------------------
figure
frame = [0, xMax, 0, yMax];
robot = DrawableRobot(25, 0, pi/2, CC.Rw, CC.L, CC.Gmax, CC.Vmax, CC.Ld);
robot.DrawRobot(frame);
hold on
[ii,jj] = find(map > occThresh);
[xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
obstclesPlot = plot(xx, yy, 'r.');

%scan = scan * 10;
for t = 0:CC.DT:(CC.T - CC.DT)
  redraw = mod(t, CC.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end
  
  if mod(t, scan) == 0
    [probGrid, occGrid] = getObstacleField(probGrid, map, CC, xMax, yMax, ...
      robot);
  end
  if redraw
    [ii,jj] = find(probGrid > occThresh);
    [xx, yy] = IJtoXY(ii, jj, xMax, yMax, R, C);
    delete(obstPlot)
    obstPlot = plot(xx, yy, 'k.');
  end
  
  rx = robot.x;
  ry = robot.y;
  rt = robot.theta;
  g = robot.RobotPoint([goalX; goalY]);
  Ft = CC.Fct / sqrt(g(X)^2 + g(Y)^2) .* [g(X), g(Y)];
  
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
      p = probGrid(I, J);
      if p > occThresh
        rp = h2e(se2(rx, ry, rt) \ e2h([x;y]));
        d = max(sqrt(rp(X)^2 + rp(Y)^2), 1e-6);
        occ = p * fcr / (d^3);
        FrX = FrX + occ * rp(X);
        FrY = FrY + occ * rp(Y);
      end
    end
  end
  Rx = FrX + Ft(X);
  Ry = FrY + Ft(Y);
  gammaD = atan2(Rx, -Ry) - robotOr;
  
  robot.MoveEulerAck_RedrawRobot(gammaD, vd, CC, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if abs(g(X)) + abs(g(Y)) < CC.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end

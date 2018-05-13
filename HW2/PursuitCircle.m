%{
 PursuitCircle.m
 Tim Ambrose
 28 Apr 2018
 Generates circle points and persues with pure persuit controller
%}
addpath '../Common'
clear
close all

alloc = 1000;          %initial allocation
C = struct('X', 1,... %X index
  'Y',    2,...       %Y index
  'dt',   0.001,...   %seconds
  'DT',   0.01,...    %seconds
  'T',    60.0,...    %total move to point time allowed
  'Rw',   0.5,...     %wheeel radius [m]
  'L',    2.5,...     %wheelbase [m]
  'Vmax', 1,...       %v max     [m/s]
  'Gmax', pi/4,...    %gamma max [radians]
  'Ld',   2,...       %min distance to first navigatable point [meters]
  'sPoints', 15,...   %straight points
  'cPoints', 63,...   %curve points
  'posEpsilon', 0.2,... %position requirement
  'aniPause', 0.02 ...  %animation pause
  );

robot = DrawableRobot(15, 5, pi/2, C.Rw, C.L, C.Gmax, C.Vmax, C.Ld);
path = zeros(2, C.cPoints);
pathparam = [9, 7, 5, -5];
angles = 0:0.1:(2 * pi);
path(C.X, :) = pathparam(1) + pathparam(3) .* sin(angles);
path(C.Y, :) = pathparam(2) + pathparam(4) .* cos(angles);
planner = PathPlanner(path);
frameSize = 14;        %14x14 meters square plot area
frame = [2, frameSize + 2, 0, frameSize];
robot.DrawRobot(frame)
pathPoint = 'b.';

hold on
nPoints = size(path, 2);
plot(path(C.X, :), path(C.Y, :), 'g*')
plot(path(C.X, 1), path(C.Y, 1), 'b*')
plot(path(C.X, nPoints), path(C.Y, nPoints), 'r*')

tauG = 0.15;
tauV = 0.5;
s = 0;
skidDR = 0;
skidDF = 0;
vd = C.Vmax;
k = 1;
stepErr = zeros(1, alloc);

prev = 1;
for t = 0:C.DT:(C.T - C.DT)
  prevPos = [robot.x, robot.y];

  if k > length(stepErr)
    stepErr = [stepErr, zeros(1, length(stepErr))];
  end
  [gammaD, err, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  stepErr(k) = err;
  k = k + 1;

  robot.MoveEulerAck_RedrawRobot(gammaD, vd, C, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint);
  if prev == planner.nPoints && abs(errX) + abs(errY) < C.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end

xlabel('World X [m]')
ylabel('World Y [m]')
figure
plot(1:k, stepErr(1:k))
xlim([0 k+10])
xlabel('DT Time Step (0.01 s)')
ylabel('Tracking Error [m]')


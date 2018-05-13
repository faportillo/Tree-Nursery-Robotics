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
  'Gmax', 45 * pi/180,... %gamma max [radians]
  'Ld',   2.9,...         %min distance to first navigatable point [meters]
  'sPoints', 15,...       %straight points
  'cPoints', 63,...       %curve points
  'posEpsilon', 0.2,...   %position requirement
  'redrawT',0.1,...       %# of DT to redraw robot for pursuit controller
  'aniPause', 0.01 ...    %animation pause
  );

robot = DrawableRobot(0, 0, 0, C.Rw, C.L, C.Gmax, C.Vmax, C.Ld);
path = zeros(2, 48);
path(C.X, :) = [linspace(0, 10, 20), (zeros(1, 8) + 10), ...
  linspace(10, 20, 20)];
path(C.Y, :) = [zeros(1, 20), linspace(0.5, 4.5, 8), (zeros(1, 20) + 5)];
planner = PathPlanner(path);
frameSize = 22;        %22x22 meters square plot area
frame = [-1, frameSize - 1, -5, frameSize - 5];
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
  redraw = mod(t, C.redrawT) == 0;
  if redraw
    prevPos = [robot.x, robot.y];
  end

  if k > length(stepErr)
    stepErr = [stepErr, zeros(1, length(stepErr))];
  end
  [gammaD, err, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  stepErr(k) = err;
  k = k + 1;
  
  robot.MoveEulerAck_RedrawRobot(gammaD, vd, C, s, skidDR, skidDF, tauV, ...
    tauG, frame, prevPos, pathPoint, redraw);
  if prev == planner.nPoints && abs(errX) + abs(errY) < C.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end

xlabel('World X [m]')
ylabel('World Y [m]')
%ylim([-1 7])
figure
plot(1:k, stepErr(1:k))
xlim([0 k+10])
xlabel('DT Time Step (0.01 s)')
ylabel('Tracking Error [m]')


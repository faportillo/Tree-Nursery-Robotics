addpath '../Common'

clear
%figure
close all

dt = [0.05, 0.1, 0.5];
pathPoints = {'r*', 'b*', 'g*'};
slipL = 0.05;
slipR = 0.1;
skid = 5;
Sl = [0, slipL, 0, 0, 0, slipL];
Sr = [0, slipR, 0, 0, 0, slipR];
skidD = [0, 0, 0, skid * pi/180, 0, skid * pi/180];

straightSpeed = 7.7;
ts = straightSpeed * 0.79;
animationPause = 0.2;

robot = DrawableRobot(0, 4, 0);
robot.Rw = 0.5;
robot.l = 1;
frameSize = 5;
frame = [-frameSize, frameSize, -frameSize, frameSize];
robot.DrawRobot(frame)

for i = 1:length(dt)
  nPoints = floor(7.8 / dt(i));
  animationStep = dt(i) * animationPause;
  for j = 1:nPoints
   pause(animationStep)
   robot.MoveGeometricDiff_RedrawRobot(-straightSpeed, -ts, dt(i), frame,...
     pathPoints{i})
  end
end

figure
robot = DrawableRobot(0, 4, 0);
robot.Rw = 0.5;
robot.l = 1;
robot.DrawRobot(frame)

for i = 1:length(dt)
  nPoints = floor(7.8 / dt(i));
  animationStep = dt(i) * animationPause;
  for j = 1:nPoints
   pause(animationStep)
   robot.MoveEulerDiff_RedrawRobot(ts, straightSpeed, dt(i), Sl(1), Sr(1),...
     skidD(1), frame, pathPoints{i})
  end
end

nPoints = floor(7.8 / dt(2)) * [1, 1.4, 1, 1, 1, 1.4];
for i = 1:length(Sl)
  if mod((i - 1), 2) == 0
    figure
    robot = DrawableRobot(0, 5, 0);
    robot.Rw = 0.5;
    robot.l = 1;
    frameSize = 7;
    frame = [-frameSize, frameSize, -frameSize, frameSize];
    robot.DrawRobot(frame)
  end
  
  animationStep = dt(2) * animationPause;
  for j = 1:nPoints(i)
   pause(animationStep)
   robot.MoveEulerDiff_RedrawRobot(ts, straightSpeed, dt(2), Sl(i), Sr(i),...
     skidD(i), frame, pathPoints{mod(i, 2) + 1})
  end
end
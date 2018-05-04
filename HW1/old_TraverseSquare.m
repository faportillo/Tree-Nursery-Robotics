addpath '../Common'

clear
%figure
%close all
dt = 1;
robot = DrawableRobot(4, 4, 0);
robot.Rw = 1;
robot.l = 2;
frameSize = 5;
frame = [-frameSize, frameSize, -frameSize, frameSize];
robot.DrawRobot(frame)

ts = 0.785;
%ts = 0.81;
WrPoints = [1, 1, 1, 1, 1, 1, 1, 1,  ts,  ts, 1, 1, 1, 1, 1, 1, 1, 1,...
   ts,  ts, 1, 1, 1, 1, 1, 1, 1, 1,  ts,  ts, 1, 1, 1, 1, 1, 1, 1, 1];
WlPoints = [1, 1, 1, 1, 1, 1, 1, 1, -ts, -ts, 1, 1, 1, 1, 1, 1, 1, 1,...
  -ts, -ts, 1, 1, 1, 1, 1, 1, 1, 1, -ts, -ts, 1, 1, 1, 1, 1, 1, 1, 1];

nPoints = length(WlPoints);
%nPoints = 32;
xPoints = [3, 2, 1, 0, -1, -2, -3, -4, -4, -4, -4, -4, -4, -4, -4, -4,...
  -3, -2, -1, 0, 1, 2, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4];
yPoints = [4, 4, 4, 4, 4, 4, 4, 4, 3, 2, 1, 0, -1, -2, -3, -4, -4, -4,...
  -4, -4, -4, -4, -4, -4, -3, -2, -1, 0, 1, 2, 3, 4];
thetaPoints = [0, 0, 0, 0, 0, 0, 0, pi/4, pi/2, pi/2, pi/2, pi/2,...
  pi/2, pi/2, pi/2, 3 * pi/4, pi, pi, pi, pi, pi, pi, pi,...
  -3 * pi/4, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2];

for i = 1:nPoints
  pause(0.5)
  %robot.MoveTo_RedrawRobot(xPoints(i), yPoints(i), thetaPoints(i), frame)
  %robot.MoveGeometric_RedrawRobot(WlPoints(i), WrPoints(i), dt, frame)
  robot.MoveEuler_RedrawRobot(WlPoints(i), WrPoints(i), dt, frame)
end
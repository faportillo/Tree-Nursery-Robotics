addpath '../Common'

clear
%figure
close all

dt = [0.05, 0.1, 0.5];
pathPoints = {'r*', 'b*', 'g*'};
slipL = 0.1;
slipR = 0.2;
skid = 5;
Sl = [0, slipL, 0, 0, 0, slipL];
Sr = [0, slipR, 0, 0, 0, slipR];
skidD = [0, 0, 0, skid * pi/180, 0, skid * pi/180];
rowSeparations = [2, 5];   %meters


straightSpeed = 4;
sides = 4;
ts = straightSpeed * 0.196;
animationPause = 0.2;
%{
robot = DrawableRobot(4, 4, pi);  %start at 4 meters right of center, 4 up
                                 %facing west
robot.Rw = 0.5;                  %wheel radius [meters]
robot.l = 1;                     %wheel separation [meters]
frameSize = 5;                   %10x10 meters square plot area
frame = [-frameSize, frameSize, -frameSize, frameSize];

robot.DrawRobot(frame)

%---------------------Part 1 - Geometric - Adjusting dt-------------------------
for i = 1:length(dt)
  sidePoints = floor(4 / dt(i));
  turnSteps = 2 / dt(i);
  animationStep = dt(i) * animationPause;
  for j = 1:sides
    for k = 1:sidePoints
      pause(animationStep)
      robot.MoveGeometricDiff_RedrawRobot(straightSpeed, straightSpeed, dt(i),...
        frame, pathPoints{i})
    end
    for k = 1:turnSteps
      robot.MoveGeometricDiff_RedrawRobot(ts, -ts, dt(i), frame, pathPoints{i})
      pause(animationStep)
    end
  end
end

%---------------------Part 1 - Euler - Adjusting dt----------------------------
figure
robot = DrawableRobot(4, 4, pi);  %start at 4 meters right of center, 4 up
                                 %facing west
robot.Rw = 0.5;                  %wheel radius [meters]
robot.l = 1;                     %wheel separation [meters]
robot.DrawRobot(frame)

for i = 1:length(dt)
  sidePoints = floor(4 / dt(i));
  turnSteps = 2 / dt(i);
  animationStep = dt(i) * animationPause;
  for j = 1:sides
    for k = 1:sidePoints
      pause(animationStep)
      robot.MoveEulerDiff_RedrawRobot(straightSpeed, straightSpeed, dt(i), Sl(1),...
        Sr(1), skidD(1), frame, pathPoints{i})
    end
    for k = 1:turnSteps
      robot.MoveEulerDiff_RedrawRobot(ts, -ts, dt(i), Sl(1), Sr(1), skidD(1),...
        frame, pathPoints{i})
      pause(animationStep)
    end
  end
end

%---------------Part 2 - Slipping and Skidding-------------------
straightSpeed = 5;      % meters per second
for i = 1:length(Sl)
  if mod((i - 1), 2) == 0
    figure
    robot = DrawableRobot(9, 4, pi);  %start at 9 meters right of center, 4 up
                                     %facing west
    robot.Rw = 0.5;                  %wheel radius [meters]
    robot.l = 1;                     %wheel separation [meters]
    frameSize = 11;                  %22x22 meters square plot area
    frame = [-frameSize, frameSize, -frameSize, frameSize];
    robot.DrawRobot(frame)
  end
  
  sidePoints = floor(3.2 / dt(2));
  turnSteps = 2 / dt(2);
  animationStep = dt(2) * animationPause;
  for j = 1:sides
    for k = 1:sidePoints
      pause(animationStep)
      robot.MoveEulerDiff_RedrawRobot(straightSpeed, straightSpeed, dt(2), Sl(i),...
        Sr(i), skidD(i), frame, pathPoints{mod(i, 2) + 1})
    end
    for k = 1:turnSteps
      robot.MoveEulerDiff_RedrawRobot(ts, -ts, dt(2), Sl(i), Sr(i), skidD(i),...
        frame, pathPoints{mod(i, 2) + 1})
      pause(animationStep)
    end
  end
end
%}

%-----------------Part 3 - Ackerman----------------------

C = struct('dt', 0.01,...       %seconds
           'DT', 0.01,...       %seconds
           'T',  6.0,...        %total move to point time allowed
           'Kv', 0.8, ...       %speed gain
           'Kh', 6,   ...       %heading gain
           'aniPause', animationPause);
pathPoint2 = {'b.', 'r.', 'g.', 'r.', 'b.', 'g.', 'b.', 'r.', 'g.'};
%{
figure
for i = 1:length(rowSeparations)
  line = rowSeparations(i) / 2;
  robot = DrawableRobot(line, -2, pi/2, 0.5, 2.5, pi/4, 5);
  %start at 1 m right of center, 2 down, facing north
  %wheel radius 0.5 m, wheel base 2.5 m, steering angle max 45 deg,
  %velocity max 5 m/s

  destX = [line, -line];     %destinations X coords
  destY = [1, 1];            %destinations Y coords
  frameSize = 5;             %10x10 meters square plot area
  frame = [-frameSize, frameSize, -frameSize, frameSize];
  robot.DrawRobot(frame)
  hold on
  r1 = plot([-line, -line + 1e-6], [-frameSize, frameSize]);
  r2 = plot([line, line + 1e-6], [-frameSize, frameSize]);
  robot.MoveToEulerAck_RedrawRobot(destX(1), destY(1), C, 0, 0,...
    0, 0, 0, frame, pathPoint2{i})
  robot.MoveToEulerAck_RedrawRobot(destX(2), destY(2), C, 0, 0,...
    0, 0, 0, frame, pathPoint2{i})
end
%}
tau = 1.2;
kv =     [0.8, 0.8, 0.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8];
figs =   [1, 0,   0, 1, 0, 0, 1, 0, 0];
s =      [0, 0, 0.5, 0, 0, 0, 0, 0, 0];
skidDR = [3, 10,  0, 0, 0, 0, 0, 0, 0] * pi/180;
skidDF = [2, 9,   0, 0, 0, 0, 0, 0, 0];
tauV =   [0, 0,   0, 0, 0, 0, tau, tau, tau];
tauG =   [0, 0,   0, 0, tau, tau*2, 0, tau, tau*2];
for i = 1:length(s)
  if figs(i) == 1
    figure
  end
  C.Kv = kv(i);
  line = rowSeparations(2) / 2;
  robot = DrawableRobot(line, -2, pi/2, 0.5, 2.5, pi/4, 5);
  %start at 1 m right of center, 2 down, facing north
  %wheel radius 0.5 m, wheel base 2.5 m, steering angle max 45 deg,
  %velocity max 5 m/s

  destX = [line, -line];     %destinations X coords
  destY = [1, 1];            %destinations Y coords
  frameSize = 5;             %10x10 meters square plot area
  frame = [-frameSize, frameSize, -frameSize, frameSize];
  robot.DrawRobot(frame)
  hold on
  if figs(i) == 1
    r1 = plot([-line, -line + 1e-6], [-frameSize, frameSize]);
    r2 = plot([line, line + 1e-6], [-frameSize, frameSize]);
  end
  robot.MoveToEulerAck_RedrawRobot(destX(1), destY(1), C, s(i), skidDR(i),...
    skidDF(i), tauV(i), tauG(i), frame, pathPoint2{i})
  robot.MoveToEulerAck_RedrawRobot(destX(2), destY(2), C, s(i), skidDR(i),...
    skidDF(i), tauV(i), tauG(i), frame, pathPoint2{i})
end

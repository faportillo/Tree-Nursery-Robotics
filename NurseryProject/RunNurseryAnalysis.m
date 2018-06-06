%{
 RunNurseryAnalysis.m
 1 June 2018
 Main file to run to perform analysis of nursery
%}

addpath '../Common'
clear
close all

PLOT_REAL_TREES = true;
PRINT_MAP = true;
PRINT_GRID = true;
vd = 4;              %max velocity [m/s]

global bitmap;
[K, x_im, y_im] = generateNursery();


if PLOT_REAL_TREES
  figure
  imagesc(x_im, y_im, flipud(bitmap)); %imagesc flips the bitmap rows, so correct this
  set(gca,'YDir','normal');
end


X = 1;
Y = 2;
THETA = 3;
GAMMA = 4;
VEL = 5;

RL = 20;             %row length [m]
Gmax = 55 * pi/180;  %steering angle max
L = 3;               %wheelbase
xMax = 50; yMax = 100;
span = pi;
start = [0,0];
endI = 2 * K + 2;  %end index of last node
tauG = 0.1;        %steeing lag
tauV = 0.2;        %velocity lag
s = 0;             %slip
skidDR = 0;        %rear skid
skidDF = 0;        %front skid

Qmax = zeros(1, 5);
Qmax(X) = Inf; Qmax(Y) = Inf; Qmax(THETA) = Inf;
Qmax(GAMMA) = Gmax; Qmax(VEL) = vd;
Qmin = -Qmax; % symmetrical negative constraints for minimum values
% Control constraints
Umax=[Gmax vd]';
Umin= -Umax;

%constants struct
C = struct('W', 3,...     %center-to-center row distance [m]
  'swX', 20, ...          %x offset of southwest corner of grid of trees
  'swY', 20, ...          %y offset of southwest corner of grid of trees
  'L', L,...              %wheelbase [m]
  'start', start, ...     %start coordinates
  'Rw', 0.5,...           %radius [m]
  'Vmax', vd,...          %v max     [m/s]
  'Gmax', Gmax,...        %gamma max [radians]
  'Ld',   2.2,...         %min distance to first navigatable point [meters]
  'dt',   0.001,...       %seconds
  'DT',   0.01,...        %seconds
  'T',    600.0,...       %total move to point time allowed
  'RL', 20, ...           %row length [m]
  'HUGE', 10^9,...        %discouraging cost
  'ptsPerMeter', 2,...    %points to plot per meter
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 20, ...   %max range of laser
  'angleSpan', span, ...  %angle span of laser sweep
  'angleStep', span/360, ... %step of laser sweep
  'occThresh', 0.5,...    %occupancy threshold
  'endI', endI,...        %number of nodes
  'K', K, ...             %tree rows
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / vd,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );

%create nodes between ends of rows and assign costs
[nodes, DMAT] = Nursery.MakeNodes(C);
robot = DrawableRobot(0, 0, pi/2, C.Rw, C.L, C.Gmax, C.Vmax, ...
  C.Ld);
nursery = Nursery(bitmap, K, C.RL, C.W, nodes, robot);

if PRINT_MAP
  figure
end
%create path to navigate to all nodes
success = nursery.PlanPath(DMAT, PRINT_MAP, C);
if success == false %genetic algorithm generated invalid path
  return;    %error is printed internally, exit program
end

frame = [-1, 50, -1, 50];
robot.dim = [1.5, 1.2];    %3 meters long, 2.4 wide
if PRINT_MAP
  robot.DrawRobot(frame)
end
pathPoints = nursery.robotPath;
planner = PathPlanner(pathPoints);
pathPoint = 'b.';

prev = 1;
lookAhead = plot(pathPoints(X, 3), pathPoints(Y, 3), 'k*');

%-----------------------MAIN MOTION LOOP---------------------------------
for t = 0:C.DT:(C.T - C.DT)
  redraw = mod(t, C.redrawT) == 0;  %whether to redraw this iteration or not
  
  [gammaD, ~, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  delete(lookAhead)
  %plot lookahead point
  lookAhead = plot(pathPoints(X, prev), pathPoints(Y, prev), 'k*');

  robot.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
  %{
  robot_odo NOT QUITE WORKING YET
  [q, odo] = robot_odo([robot.x; robot.y; robot.theta; robot.gamma; robot.v],...
    [gammaD; vd], Umin, Umax, Qmin, Qmax, L, tauG, tauV);
  robot.x = q(X);
  robot.y = q(Y);
  robot.theta = q(THETA);
  robot.gamma = q(GAMMA);
  robot.v = q(VEL);
  %}
  if redraw
    prevPos = [robot.x, robot.y];
    robot.RedrawRobot(frame, prevPos, pathPoint)
    pause(C.aniPause)
  end
    
  if prev == planner.nPoints && abs(errX) + abs(errY) < C.posEpsilon
    break   % stop if navigating to last path point and position close enough
  end
end
  
if PRINT_MAP
  %{
  plot(pathPoints(X, 1:21), pathPoints(Y, 1:21), 'g.')
  plot(pathPoints(X, (nPoints+1):(nPoints+25)), ...
    pathPoints(Y, (nPoints+1):(nPoints+25)), 'k.')
  %}
  xlabel('World X [m]')
  ylabel('World Y [m]')
end

%test of output functions    -Tim
diam = 0.3 .* rand(20 * K, 1) + 0.2;
for i = 1:(20*K)
  nursery.AddTree(i, i*2, diam(i), floor((i-1) / 20) + 1)
end
nursery.WriteResults('testResults.txt');

fprintf('))) Done (((\n')

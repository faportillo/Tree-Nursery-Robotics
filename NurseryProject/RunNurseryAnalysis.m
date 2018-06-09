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
% PRINT_GRID = true;
vd = 4;              %max velocity [m/s]

global bitmap;
global dT;
global DT;
global occ_grid;
global prob_grid;
global Pfree; global Pocc;

[K, x_im, y_im] = generateNursery();

if PLOT_REAL_TREES
  figure(1)
  imagesc(x_im, y_im, flipud(bitmap)); %imagesc flips the bitmap rows, so correct this
  set(gca,'YDir','normal');
end


X = 1;
Y = 2;
THETA = 3;
GAMMA = 4;
VEL = 5;

RL = 20;             %row length [m]
W = 3;               % row width [m]
Gmax = 55 * pi/180;  %steering angle max
L = 3;               %wheelbase
span = deg2rad(180); % 180 degrees
start = [2,2];
endI = 2 * K + 2;  %end index of last node
tauG = 0.1;        %steeing lag
tauV = 0.2;        %velocity lag
s = 0;             %slip
skidDR = 0;        %rear skid
skidDF = 0;        %front skid
dT = 0.001;
DT = 0.01;

Qmax = zeros(1, 5);
Qmax(X) = Inf; Qmax(Y) = Inf; Qmax(THETA) = Inf;
Qmax(GAMMA) = Gmax; Qmax(VEL) = vd;
Qmin = -Qmax; % symmetrical negative constraints for minimum values
% Control constraints
Umax=[Gmax vd]';
Umin= -Umax;

% initialize as empty
r = size(bitmap,1); c = size(bitmap,2); %numbers of rows and columns of the laser scanned map (prob_grid)
occ_grid = ones(size(bitmap));
prob_grid = zeros(size(bitmap));
Pocc = 0.7; Pfree = 1-Pocc;
po = Pocc/(1-Pocc);
pf = Pfree/(1-Pfree);

%constants struct
C = struct('W', W,...     %center-to-center row distance [m]
  'swX', 20-W/2, ...      %x offset of southwest corner of grid of trees
  'swY', 20, ...          %y offset of southwest corner of grid of trees
  'L', L,...              %wheelbase [m]
  'start', start, ...     %start coordinates
  'Rw', 0.5,...           %radius [m]
  'Vmax', vd,...          %v max     [m/s]
  'Gmax', Gmax,...        %gamma max [radians]
  'Ld',   2.2,...         %min distance to first navigatable point [meters]
  'dt',   dT,...          %seconds
  'DT',   DT,...          %seconds
  'T',    600.0,...       %total move to point time allowed
  'RL', 20, ...           %row length [m]
  'HUGE', 10^9,...        %discouraging cost
  'ptsPerMeter', 2,...    %points to plot per meter
  'posEpsilon', 0.2,...   %position requirement
  'rangeMax' , 20, ...   %max range of laser
  'angleSpan', span, ...  %angle span of laser sweep
  'angleStep', deg2rad(0.125), ... %step of laser sweep
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
robot = DrawableRobot(start(X), start(Y), pi/2, C.Rw, C.L, C.Gmax, C.Vmax, ...
  C.Ld);
nursery = Nursery(bitmap, K, C.RL, C.W, nodes, robot);

if PRINT_MAP
  figure(2)
end

%create path to navigate to all nodes
success = nursery.PlanPath(DMAT, PRINT_MAP, C);
if success == false %genetic algorithm generated invalid path
  return;    %error is printed internally, exit program
end


%frame = [-1, (C.swY + C.RL + 10), -1, (C.swX + C.W * C.K + 10)];
frame = [x_im, y_im];
% define map dimensions
%frameSize = max(abs(frame(2) - frame(1)), abs(frame(4) - frame(3)));
Xmax = x_im(2); Ymax = y_im(2);
robot.dim = [1.5, 1.2];    %3 meters long, 2.4 wide
if PRINT_MAP
  figure(2)
  robot.DrawRobot(frame)
end
pathPoints = nursery.robotPath;
planner = PathPlanner(pathPoints);
pathPoint = 'b.';

prev = 1;
if PRINT_MAP
  lookAhead = plot(pathPoints(X, 3), pathPoints(Y, 3), 'k*');
end
rescanT = 10 * C.redrawT;

%-----------------------MAIN MOTION LOOP---------------------------------
for t = 0:C.DT:(C.T - C.DT)
  redraw = mod(t, C.redrawT) == 0;  %whether to redraw this iteration or not
  scan = mod(t, rescanT) == 0;
  
  [gammaD, ~, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  if PRINT_MAP
    figure(2)
    delete(lookAhead)
    %plot lookahead point
    lookAhead = plot(pathPoints(X, prev), pathPoints(Y, prev), 'k*');
  end

  %robot.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
  
  q0 = [robot.x; robot.y; robot.theta; robot.gamma; robot.v];
  
  if scan
%      for k = 1:2 % scanner rotates 90 degrees to the left and right - not sure this is allowed
%      rotate = [-pi/2 pi/2];
      Tl = se2(robot.x, robot.y, robot.theta);
      p = laserScannerNoisy(C.angleSpan, C.angleStep, C.rangeMax, Tl, bitmap, Xmax, Ymax); 
      p(:,2) = medfilt1(p(:,2));
      for j=1:length(p)
        angle = p(j,1); range = p(j,2);
        n = updateLaserBeamGrid(angle, range, Tl, r, c, C.rangeMax, Xmax, Ymax, po, pf);
      end
      figure(2)
      imagesc([frame(1) Xmax], [frame(3) Ymax], flipud(prob_grid)); %imagesc flips the bitmap rows, so correct this
      set(gca,'YDir','normal');
      pause(C.aniPause)
%      end +rotate(k)
  end
  
  u = [gammaD; vd];
  [q, odo] = robot_odo(q0, u, Umin, Umax, Qmin, Qmax, L, tauG, tauV);
  robot.x = q(X);
  robot.y = q(Y);
  robot.theta = robot.theta - (q(THETA) - robot.theta);
  robot.gamma = q(GAMMA);
  robot.v = q(VEL);
  
  if redraw && PRINT_MAP
    figure(2)
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

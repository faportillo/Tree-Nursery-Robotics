%{
 RunNurseryAnalysis.m
 1 June 2018
 Main file to run to perform analysis of nursery
%}

addpath '../Common'
addpath '../Common/geom2d'
addpath '../Common/geom2d/geom2d'
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
%% Find covariance matrices
N = 10000; 
wx = zeros(1,N);
wy = zeros(1,N);
wtheta = zeros(1,N);
odo = zeros(N,2);
q = [0 0 0 0 0]; %initialize a robot w/zero for all states.
u = [0 0]; %initialize zero for steering and speed inputs.
for i=1:N
    [wx(i), wy(i), wtheta(i)] = GPS_CompassNoisy(q(X), q(Y), q(THETA));
    [trash, odo(i,:)] = robot_odo(q, u, Umin, Umax, Qmin, Qmax, L, tauG, tauV); 
end
A = [wx; wy; wtheta]';
B = [odo(:,1) odo(:,2)];
W = cov(A); %covariance of sensor noise
V = cov(B); %covariance of odometry noise
%Initialize uncertainty matrix
P = zeros(3,3);
%initialize starting pose for the robot
kPose = [start(X); start(Y); pi/2];
q0 = [robot.x; robot.y; robot.theta; robot.gamma; robot.v];

%-----------------------MAIN MOTION LOOP---------------------------------
for t = 0:C.DT:(C.T - C.DT)
  redraw = mod(t, C.redrawT) == 0;  %whether to redraw this iteration or not
  scan = mod(t, rescanT) == 0;

  
  if scan %&& false    %temporary to suppress scanning
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
  end
  
  %robot.Move_EulerAckFK(gammaD, vd, C, s, skidDR, skidDF, tauV, tauG);
  savePose = [robot.x, robot.y, robot.theta];
  [xsensed, ysensed, thetasensed] = GPS_CompassNoisy(savePose(X), ...
    savePose(Y), savePose(THETA));
   
  z = [xsensed; ysensed; thetasensed];
  
  [kPose, P] = nurseryEKF(kPose, odo, z, P, V, W);
  %printf diff between true pose and estimate
  %{
  fprintf('%2.2f, %2.2f, D:%2.2f   %2.2f, %2.2f, D:%2.2f   %2.2f, %2.2f, D:%2.2f\n',...
    savePose(X), kPose(X), savePose(X) - kPose(X), savePose(Y), ...
    kPose(Y), savePose(Y) - kPose(Y), savePose(THETA), kPose(THETA), ...
    savePose(THETA) - kPose(THETA));
  %}
  
  %use estimated position for navigation controller
  robot.x = kPose(X);
  robot.y = kPose(Y);
  robot.theta = kPose(THETA);
  
  [gammaD, ~, prev, errX, errY] = planner.FirstFeasiblePoint(robot, prev);
  if PRINT_MAP
    figure(2)
    delete(lookAhead)
    %plot lookahead point
    lookAhead = plot(pathPoints(X, prev), pathPoints(Y, prev), 'k*');
  end
  
  %use true pose for motion simulation
  q0 = [savePose(X); savePose(Y); savePose(THETA); robot.gamma; robot.v];
  u = [gammaD; vd];
  [q, odo] = robot_odo(q0, u, Umin, Umax, Qmin, Qmax, L, tauG, tauV);
  
  %set robot to new position for drawing
  robot.x = q(X); 
  robot.y = q(Y);
  robot.theta = savePose(THETA) - (q(THETA) - savePose(THETA)); %q(THETA); %
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

%Process prob_grid and add trees to nursery tree list
nursery.ProcessGrid(prob_grid, Xmax, Ymax, r, c);

nursery.WriteResults('testResults.txt');

fprintf('))) Done (((\n')
%{
 PursuitCircle.m
 Tim Ambrose
 28 Apr 2018
 Generates circle points and persues with pure persuit controller
%}

addpath '../Common'
clear
close all

RL = 20;      %meters
Gmax = 60 * pi/180;
L = 3;
X = 1;
Y = 2;
N = 10;
Vm = 4;

initAlloc = 1000;
endI = 2 * N + 2;
C = struct('W', 2.5,...   %row width [m]
  'L', L,...              %wheelbase [m]
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
  'ptsPerMeter', 2,...    %
  'posEpsilon', 0.2,...   %position requirement
  'endI', endI,...        %number of nodes
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 5, ...          %multiplier
  'redrawT',0.2 / Vm,...  %# of DT to redraw robot for pursuit controller
  'aniPause', 0.001 ...   %animation pause
  );
path = zeros(2, 3);
planner = PathPlanner(path, C.Rmin, C.W);
xs = (C.W / 2):C.W:(C.W / 2 + C.W * (C.N - 1));
robot = DrawableRobot(-4 * C.W, RL / 2, pi/2, C.Rw, C.L, C.Gmax, C.Vmax, C.Ld);
nodes = [robot.x, xs, xs, robot.x;
  robot.y, zeros(1, C.N), (zeros(1, C.N) + RL), robot.y];

fprintf('Start and End: (%0.2f, %0.2f) (%0.2f, %0.2f)\n', nodes(X, 1), nodes(Y, 1),...
  nodes(X, endI), nodes(Y, endI))
fprintf('Bottom Nodes: ')
for i = 2:(C.N + 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\nTop Nodes: ')
for i = (C.N + 2):(endI - 1)
  fprintf('(%0.2f, %0.2f) ', nodes(X, i), nodes(Y, i))
end
fprintf('\n')

DMAT = zeros(endI);

for i = 2:(C.N + 1)               %all lower nodes
  for j = (C.N + 2):(2 * C.N + 1) %visit upper nodes
    if (j - i) == C.N             %nodes are in same row
      DMAT(i, j) = -C.MULT;       %cost to go through row is 0
      DMAT(j, i) = -C.MULT;
    else                    %diagonal traversal not allowed
      DMAT(i, j) = C.HUGE;  %cost is unreasonably large
      DMAT(j, i) = C.HUGE;
    end
  end
end

%headlands turning costs
for i = 2:C.N
  for j = (i + 1):(C.N + 1)   
    d = abs(i - j);
    if d * C.W >= 2 * C.Rmin   %Pi turn
      DMAT(i, j) = C.MULT * (d * C.W + (pi - 2) * C.Rmin);
    else                       %Omega turn
      DMAT(i, j) = C.MULT * (3 * pi * C.Rmin - 2 * C.Rmin *...
        acos(1 - (2 * C.Rmin + d * C.W) ^ 2 / (8 * C.Rmin ^ 2)));
    end
    
    DMAT(j, i) = DMAT(i, j);
    DMAT(i + C.N, j + C.N) = DMAT(i, j);
    DMAT(j + C.N, i + C.N) = DMAT(i, j);
  end
end

%Start and end points
for j = 2:(endI - 1)
  DMAT(1, j) = abs(nodes(X, 1) - nodes(X, j)) + abs(nodes(Y, 1) - nodes(Y, j));
  DMAT(j, 1) = DMAT(1, j);
  DMAT(endI, j) = abs(nodes(X, endI) - nodes(X, j)) + ...
    abs(nodes(Y, endI) - nodes(Y, j));
  DMAT(j, endI) = DMAT(endI, j);
end

DMAT(1, endI) = C.HUGE;
DMAT(endI, 1) = C.HUGE;

XY = [nodes(X, :) nodes(Y, :)];
tic;
results = tspof_ga('xy', nodes', 'DMAT', DMAT, 'SHOWRESULT', false,...
  'SHOWWAITBAR', false, 'SHOWPROG', false);
E = toc;
fprintf('Optimization took %f seconds.\n', E);

route = [1 results.optRoute endI];
fprintf('Minimum Distance: %f\n', results.minDist)
fprintf('Route: ')
disp(route)
fprintf('\n')
bad = ValidPath(route, C.N);
pathPoints = zeros(2, initAlloc);
nPoints = 21;
upY =   linspace(0, RL, RL * C.ptsPerMeter);
downY = linspace(RL, 0, RL * C.ptsPerMeter);

if bad == 0
  fprintf('Good Route\n')
  
  pathPoints(:, 1) = nodes(:, 1);
  pathPoints(:, 2:nPoints) = [(zeros(1, 10) +  nodes(X, 1)), ...
    linspace(nodes(X, 1), nodes(X, route(2)), 10); linspace(nodes(Y, 1), ...
    nodes(Y, route(2)), 10), (zeros(1, 10) +  nodes(Y, route(2)))];
  
  for node = 3:2:(endI - 1)
    nI = route(node);
    nI_1 = route(node + 1);
    bottom = nodes(Y, nI) == 0;
    left = nodes(X, nI) > nodes(X, nI_1);
    if bottom && left
      type = 'bl';
      yCoords = downY;
    elseif bottom && ~left
      type = 'br';
      yCoords = downY;
    elseif ~bottom && left
      type = 'tl';
      yCoords = upY;
    else
      type = 'tr';
      yCoords = upY;
    end
    
    straight = [zeros(1, RL * C.ptsPerMeter) + nodes(X, nI); yCoords]; 
    if node ~= endI - 1          
      points = planner.GenerateTurnPath(C.ptsPerMeter, nodes(X, nI), ...
        nodes(Y, nI), type, abs(route(node) - route(node + 1)));
    end
    
    oldNPoints = nPoints;
    if node == endI - 1
      nPoints = length(straight) + nPoints;
    else
      nPoints = length(points) + length(straight) + nPoints;
    end
    
    if nPoints > length(pathPoints)     %reallocate logarithmically
      pathPoints = [pathPoints, zeros(2, length(pathPoints))];
    end
    %add points to full path
    pathPoints(:, (oldNPoints + 1):(oldNPoints + length(straight))) = ...
      straight;
    
    if node ~= endI - 1
      pathPoints(:, (nPoints - length(points) + 1):nPoints) = points;
    end

  end
  
  if nPoints+25 > length(pathPoints)     %reallocate logarithmically
    pathPoints = [pathPoints, zeros(2, length(pathPoints))];
  end
  pathPoints(:, (nPoints+1):(nPoints+25)) = [linspace(nodes(X, route(...
    endI - 1)), nodes(X, 1), 10), (zeros(1, 15) + nodes(X, 1)); ...
    (zeros(1, 10) + nodes(Y, route(endI - 1))), linspace(nodes(Y, ...
    route(endI - 1)), nodes(Y, 1), 15)];
  
  for i = 0:C.N
    plot(C.W * [i,(i+1e-6)], [0,RL], 'r-')
    hold on
  end
  plot(pathPoints(X, 22:nPoints), pathPoints(Y, 22:nPoints), 'b.')
  
  planner1 = PathPlanner(pathPoints(:, 1:(nPoints+25)));
  frame = [-15.5, 30, -12.5, 33];
  if nodes(Y, route(2)) == 0
    robot.theta = -pi/2;
  end
  robot.DrawRobot(frame)
  pathPoint = 'b.';
  
  tauG = 0.15;
  tauV = 0.5;
  s = 0;
  skidDR = 0;
  skidDF = 0;
  vd = C.Vmax;
  k = 1;
  stepErr = zeros(1, initAlloc);

  prev = 1;
  lookAhead = plot(pathPoints(X, 3), pathPoints(Y, 3), 'k*');
  for t = 0:C.DT:(C.T - C.DT)
    redraw = mod(t, C.redrawT) == 0;
    if redraw
      prevPos = [robot.x, robot.y];
    end

    if k > length(stepErr)
      stepErr = [stepErr, zeros(1, length(stepErr))];
    end
    [gammaD, err, prev, errX, errY] = planner1.FirstFeasiblePoint(robot, prev);
    delete(lookAhead)
    lookAhead = plot(pathPoints(X, prev), pathPoints(Y, prev), 'k*');
    stepErr(k) = err;
    k = k + 1;

    robot.MoveEulerAck_RedrawRobot(gammaD, vd, C, s, skidDR, skidDF, tauV, ...
      tauG, frame, prevPos, pathPoint, redraw);
    if prev == planner1.nPoints && abs(errX) + abs(errY) < C.posEpsilon
      break   % stop if navigating to last path point and position close enough
    end
  end
 
  plot(pathPoints(X, 1:21), pathPoints(Y, 1:21), 'g.')
  plot(pathPoints(X, (nPoints+1):(nPoints+25)), ...
    pathPoints(Y, (nPoints+1):(nPoints+25)), 'k.')

  xlabel('World X [m]')
  ylabel('World Y [m]')

else
  fprintf('Bad Pairs: %d\nTry Running Again...\n', bad)
end

fprintf('Done\n')
      
%{
 PursuitCircle.m
 Tim Ambrose
 28 Apr 2018
 Generates circle points and persues with pure persuit controller
%}

addpath '../Common'

useFishtail = false;
RL = 20;      %meters
Gmax = 60 * pi/180;
L = 3;
X = 1;
Y = 2;
N = 10;

initAlloc = 500;
endI = 2 * N + 2;
C = struct('W', 2.5,...   %row width [m]
  'L', L,...              %wheelbase [m]
  'Rw', 0.5,...           %radius [m]
  'Vmax', 1,...           %v max     [m/s]
  'Gmax', Gmax,...        %gamma max [radians]
  'N', N,...              %crop rows
  'RL', 20, ...           %
  'HUGE', 10^9,...        %
  'pointsPerMeter', 2,... %
  'endI', endI,...        %number of nodes
  'Rmin', L / tan(Gmax),... %Min turning radius
  'MULT', 500 ...           %multiplier
  );
path = zeros(2, 3);
planner = PathPlanner(path, C.Rmin, C.W);

robot = DrawableRobot(-C.W, RL / 2, pi/2, C.Rw, C.L, C.Gmax, C.Vmax);
nodes = [-C.W, (C.W / 2 + 0:(C.N + 1)), (C.W / 2 + 0:(C.N + 1)), -C.W;...
         (RL / 2), zeros(1, C.N), (zeros(1, C.N) + RL), (RL / 2)];

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
bad = ValidPath(route, C);
pathPoints = zeros(2, initAlloc);
nPoints = 0;
upY =   linspace(0, RL, RL * C.pointsPerMeter);
downY = linspace(RL, 0, RL * C.pointsPerMeter);
if bad == 0
  fprintf('Good Route\n')
  for node = 3:2:(endI - 2)
    nI = route(node);
    bottom = nodes(Y, nI) == 0;
    left = nodes(X, nI) > nodes(X, nI + 1);
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
    
    straight = [zeros(1, RL * 2) + nodes(X, nI); yCoords]; 
              
    points = planner.GenerateTurnPath(useFishtail, nodes(X, nI), ...
      nodes(Y, nI), type, abs(nodes(X, nI) - nodes(X, nI + 1)));
    
    nPoints = length(points) + nPoints;
    if nPoints > length(pathPoints)     %reallocate logarithmically
      pathPoints = [pathPoints, zeros(2, length(pathPoints))];
    end
    %add points to full path
    pathPoints((nPoints - length(points) + 1):nPoints) = points;
    
    
  end
else
  fprintf('Bad Pairs: %d\n', bad)
end

fprintf('Done\n')
      
function success = PlanPath(obj, DMAT, plotPath, C)  

X = 1;
Y = 2;
endI = C.endI;
initAlloc = 1000;
nodes = obj.rowNodes;
swX = C.swX; swY = C.swY;
planner = PathPlanner(zeros(2,3), C.Rmin, C.W);

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
bad = ValidPath(route, C.K);
pathPoints = zeros(2, initAlloc);
toP = 29;
fromP = 34;
nPoints = toP + 1;
upY =   linspace(swY, C.RL + swY, C.RL * C.ptsPerMeter);
downY = linspace(C.RL + swY, swY, C.RL * C.ptsPerMeter);

if bad == 0
  fprintf('Good Route\n')
  success = true;
  
  %first point is first node
  pathPoints(:, 1) = nodes(:, 1);
  %square path to start point               TODO: Tim may make this a pi turn
  horiz = nodes(Y, route(2));
  if horiz == swY
    hLine = swY - 6;
  else
    hLine = swY + C.RL + 6;
  end
  pathPoints(:, 2:nPoints) = [(zeros(1, 15) +  nodes(X, 1)), ...
    linspace(nodes(X, 1), nodes(X, route(2)), 10), ...
    (zeros(1, 4) + nodes(X, route(2))); ...
    linspace(nodes(Y, 1), hLine, 15), (zeros(1, 10) +  hLine), ...
    linspace(hLine, horiz, 4)];
  
  for node = 3:2:(endI - 1)
    nI = route(node);
    nI_1 = route(node + 1);
    bottom = nodes(Y, nI) == swY;
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
    
    straight = [zeros(1, C.RL * C.ptsPerMeter) + nodes(X, nI); yCoords]; 
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
  
  %square path to end point               TODO: Tim may make this a pi turn
  if nPoints+25 > length(pathPoints)     %reallocate logarithmically
    pathPoints = [pathPoints, zeros(2, length(pathPoints))];
  end
  horiz = nodes(Y, route(endI - 1));
  if horiz == swY
    hLine = swY - 6;
  else
    hLine = swY + C.RL + 6;
  end
  pathPoints(:, (nPoints+1):(nPoints+fromP)) = [(zeros(1, 4) + nodes(X, ...
    route(endI - 1))), linspace(nodes(X, route(endI - 1)), nodes(X, 1), 10), ...
    (zeros(1, 20) + nodes(X, 1)); ...
    linspace(nodes(Y, route(endI - 1)), hLine, 4), (zeros(1, 10) + hLine), ...
    linspace(hLine, nodes(Y, 1), 20)];
  
  if plotPath
    for i = 0:C.K
      plot(C.W * [i,(i+1e-6)] + swX, swY + [0,C.RL], 'r-')
      hold on
    end
    
    plot(pathPoints(X, (toP + 2):nPoints), pathPoints(Y, (toP + 2):nPoints), ...
      'b.')
    plot(pathPoints(X, 1:(toP + 1)), pathPoints(Y, 1:(toP + 1)), 'g.')
    plot(pathPoints(X, (nPoints+1):(nPoints+fromP)), ...
      pathPoints(Y, (nPoints+1):(nPoints+fromP)), 'k.')
  end
  obj.robotPath = pathPoints(:, 1:(nPoints+fromP));
  
else
  fprintf('Bad Pairs: %d\nTry Running Again...\n', bad)
  success = false;
end

end
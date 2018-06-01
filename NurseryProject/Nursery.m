classdef Nursery < handle
  properties
    gtMap               %ground truth map of trees
    nRows               %number of tree rows
    rowW                %center-to-center distance of tree rows
    rowL                %
    rowNodes            %points marking the tree rows end- and midpoints
    robotPath           %path to traverse the nursery
    robot               %robot instance
  end
  properties (SetAccess = private)
    nodeRoute
  end
  methods
    function obj = Nursery(map, K, rowLength, rowWidth, nodes, robo)
      obj.gtMap = map;
      obj.nRows = K;
      obj.rowL = rowLength;
      obj.rowW = rowWidth;
      obj.rowNodes = nodes;
      obj.robot = robo;
    end
    
    function valid = PlanPath(obj, C)
      %XY = [rowNodes(X, :) rowNodes(Y, :)];
      tic;
      results = tspof_ga('xy', obj.rowNodes', 'DMAT', DMAT, 'SHOWRESULT', ...
        false, 'SHOWWAITBAR', false, 'SHOWPROG', false);
      E = toc;
      fprintf('Optimization took %f seconds.\n', E);

      obj.nodeRoute = [1 results.optRoute endI];
      fprintf('Minimum Distance: %f\n', results.minDist)
      valid = ValidPath(obj.nodeRoute, C);
    end
    
    %declared in separate file
    nodes = MakeNodes(obj)
  end
end
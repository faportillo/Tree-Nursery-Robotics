classdef Nursery < handle
  properties
    gtMap               %ground truth map of trees
    nRows               %number of tree rows
    rowW                %center-to-center distance of tree rows
    rowL                %
    rowNodes            %points marking the tree rows end- and midpoints
    nNodes              %number of nodes
    robotPath           %path to traverse the nursery
    robot               %robot instance
  end
  properties (SetAccess = private)
    nodeRoute
    treeMeasurements
    treesInRow
    X_                  %index of X coordinate for tree
    Y_                  %index of Y coordinate for tree
    D_                  %index of diameter of tree
    allocTreesInRow
  end
  methods
    function obj = Nursery(map, K, rowLength, rowWidth, nodes, robo)
      obj.allocTreesInRow = 10;
      obj.gtMap = map;
      obj.nRows = K;
      obj.rowL = rowLength;
      obj.rowW = rowWidth;
      obj.rowNodes = nodes;
      obj.robot = robo;
      obj.treeMeasurements = cell(1, K);
      obj.treesInRow = zeros(1, K);
      obj.X_ = 1;
      obj.Y_ = 2;
      obj.D_ = 3;
      for i = 1:K
        %allocate space for x,y,diameter for each tree in row
        obj.treeMeasurements{i} = zeros(obj.allocTreesInRow, 3);
      end
    end
    
    function valid = PlanRoute(obj, C)
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
    
    function success = WriteResults(obj, filename)
      x = obj.X_;
      y = obj.Y_;
      d = obj.D_;
      fileID = fopen(filename, 'w');
      if fileID >= 0
        for r = 1:obj.nRows
          fprintf(fileID, '%d\n', r);
          for tree = 1:obj.treesInRow(r)
            meas = obj.treeMeasurements{r}(tree, :);
            fprintf(fileID, '%d,%0.2f,%0.2f,%0.2f\n', tree, meas(x), meas(y), meas(d));
          end
          fprintf(fileID, '\n');
        end
        success = true;
      else
        success = false;
      end
      fclose(fileID);
    end
    
    function AddTree(obj, x, y, diameter, row)
      inRow = obj.treesInRow(row) + 1;
      obj.treesInRow(row) = inRow;
      if inRow > size(obj.treeMeasurements{row}, 1)
        obj.treeMeasurements{row} = [obj.treeMeasurements{row}; ...
                                     zeros(inRow - 1, 3)];
      end
      obj.treeMeasurements{row}(inRow, :) = [x, y, diameter];
      %make sure trees are sorted by y coordinate within each row list
      obj.treeMeasurements{row}(1:inRow, :) = ...
        sortrows(obj.treeMeasurements{row}(1:inRow, :), 2);
    end
    
    %defined in separate file
    success = PlanPath(obj, DMAT, plotPath, C)
    ProcessGrid(obj, grid, Xmax, Ymax, R, C)
  end
  methods (Static)
    %defined in separate file
    [nodes, DMAT] = MakeNodes(C)
    %{
    function [K, map] = InterpretInput(file, C)
      K = 10;           %temperary value until file format is known
      map = zeros(C.C, C.R);
    end
    %}
  end
end
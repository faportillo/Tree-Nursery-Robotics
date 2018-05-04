classdef PathPlanner < handle
  properties
    %;
  end
  properties (SetAccess = private)
    path_         %Defined as [x0 x1 x2 ... ]
                  %           [y0 y1 y2 ... ]
    nPoints       %number of coordinate pairs in path
  end
  methods
    function obj = PathPlanner(varargin)
      %{1} = path
      if nargin > 0
        obj.path_ = varargin{1};
        obj.nPoints = size(obj.path_, 2);
      else
        obj.path_ = zeros(2, 3);
        obj.nPoints = 3;
      end
    end
    
    %Pure Pursuit: Find first point in front of robotf
    function [steerAngle, trackErr, first, Ex, Ey] = FirstFeasiblePoint(obj,...
        robot, previousIndex)
      trackErr = Inf;
      first = previousIndex;
      startMin = Inf;
      X = 1; Y = 2;
      for i = 1:obj.nPoints
        di = sqrt(abs(obj.path_(X, i) - robot.x) ^ 2 + ...
                  abs(obj.path_(Y, i) - robot.y) ^ 2 );
        if di < trackErr
          trackErr = di;
        end
        if i >= previousIndex
          pointD = abs(di - robot.Ld);
          if robot.inFront(obj.path_(:, i)) && pointD < startMin
            startMin = pointD;
            first = i;
          end
        end
      end
      
      
      Pr = robot.RobotPoint(obj.path_(:, first));
      Ey = Pr(X);
      Ex = -Pr(Y);
      steerAngle = atan(robot.l * 2 * Ey / (robot.Ld ^ 2));
      if first ~= previousIndex
        fprintf('.')
      end
    end
    
    %--------------GETTERS and SETTERS-------------------
    function path = Path(obj)
      path = obj.path_;
    end
    function newPath(obj, path)
      obj.path_ = path;
      obj.nPoints = length(path, 2);
    end
  end
end
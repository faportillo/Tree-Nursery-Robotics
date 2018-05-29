function [oddsGrid, occGrid] = getObstacleField(map, C, xMax, yMax, robot)
  global rangeMax;

  occGrid = zeros(C.R, C.C); %initialize as empty
  oddsGrid = ones(C.R, C.C);
  rangeMax = C.rangeMax;

  Tl = se2([robot.x robot.y robot.theta]);
  p = laserScanner(C.angleSpan, C.angleStep, rangeMax, Tl, map, xMax, yMax);
  for i=1:length(p)
    angle = p(i,1); range = p(i,2);
    occGrid = updateLaserBeamBitmap(occGrid, angle, range, Tl, C.R, C.C, ...
      xMax, yMax);
    oddsGrid = UpdateLaserBeamGrid(oddsGrid, angle, range, Tl, C.R, C.C, ...
      xMax, yMax);
  end

end
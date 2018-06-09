% function assumes a laser scanner with a pose in world coordinates defined by Tl
% has shot a ray from some 'angle' and returned 'range'. If no obstacle
% within range, laser returned a value equal to Inf.
% An occupancy grid has pixels that contain odds.
% The grid has origin at the SW corner(0,0) and extends to the NE corner (Xmax, Ymax),
% The pixel odds along the laser beam, from source to 'range', at 'angle', are updated.
function p = updateLaserBeamGrid(angle, range, Tl, R, C, rangeMax, Xmax, Ymax, po, pf)
global occ_grid;
%global Pfree; global Pocc;

global prob_grid;

%transform laser origin to world frame
P1 = Tl*[0 0 1]';
x1=P1(1);     y1=P1(2);
% calculate corresponding pixel in grid
[ I1, J1 ] = XYtoIJ(x1, y1, Xmax, Ymax, R, C); % laser source

if isinf(range) == 1
range = Xmax^2+Ymax^2;  % assign arbitrary huge value
end

%first produce target point for laser in scanner frame
Xl = range * cos(angle);
Yl = range * sin(angle);

%Transform target point in world frame
P2 = Tl*[Xl Yl 1]';
x2=P2(1); y2=P2(2);

%clip laser beam to boundary polygon so that 'infinite' (rangeMax) range
% extends up to the boundary
dy = y2-y1; dx = x2-x1;
% ATTENTION: if dy or dx is close to 0 but negative, make it almost zero
% positive
if (abs(y2-y1)) < 1E-6
    dy = 1E-6;
end
edge = clipLine([x1,y1,dx,dy],[0 Xmax 0 Ymax]);
%laser origin is always inside map
%decide if clipping is necessary
l1 = sqrt( (x1-edge(3))^2 + (y1 - edge(4))^2);
if range >= l1
    x2 = edge(3); y2 = edge(4);
end

% map world points to integer (grid) coordninates
[ I2, J2 ] = XYtoIJ(x2, y2, Xmax, Ymax, R, C); % obstacle
if (I2<1) || (J2<1) || I2>R || J2>C
    disp('bitmap index out of range'); exit;
end
%update odds for detected target pixel; measurement suggests it is occupied. 
occ_grid(I2, J2) = occ_grid(I2, J2) * po;
%if range < rangeMax
  prob_grid(I2, J2) = occ_grid(I2, J2)/(1+occ_grid(I2, J2)); % update with probability
%end
% use bresenham to find all pixels that are between laser and obstacle;
% measurement suggests they are in free space (no obstacles)
l=bresenhamFast(I1,J1,I2,J2);
%[l1 l2]=size(l);
for k=1:length(l)-1 %skip the target pixel
  occ_grid(l(k,1),l(k,2)) = occ_grid(l(k,1),l(k,2)) * pf;
 % if range < rangeMax
    prob_grid(l(k,1),l(k,2)) = occ_grid(l(k,1),l(k,2)) / ...
      (1+occ_grid(l(k,1),l(k,2))); % update with probability
 % end
end

p = length(l) + 1;  % number of updated pixels


end


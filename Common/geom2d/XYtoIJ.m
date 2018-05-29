function [ i, j ] = XYtoIJ(x, y, Xmax, Ymax, R, C)
% compute pixel map coordinates from World cartesian coordinates
%   Detailed explanation goes here
i = round(((Ymax - y) ./ Ymax) .* (R-1)) + 1; 
j = round((x ./ Xmax) .* (C-1)) + 1;

end


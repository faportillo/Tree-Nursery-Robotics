function tree_list = process_grid(grid)
%PROCESS_GRID Processes Occupancy Grid
%   Detailed explanation goes here
    boundaries = bwboundaries(grid);
    n = length(boundaries);
    imshow(grid);
    hold on
    for k=1:n
        b=boundaries{k};
        diameter = abs(max(b(:,2))-min(b(:,1)))
        plot(b(:,2),b(:,1),'g','LineWidth',3);
    end
    tree_list = [];
end


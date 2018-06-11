function nursery = process_grid(grid,nursery,Xmax,Ymax,R,C)
%PROCESS_GRID Processes Occupancy Grid
%   grid: Occupancy Grid
%   nursery: Nursery Object, use AddTree Function
    boundaries = bwboundaries(grid);
    n = length(boundaries);
    imshow(grid);
    hold on
    for k=1:n
        b=boundaries{k};
        %Convert pixel to XY Points
        [bx_max,by_max] = IJtoXY(max(b(:,2),max(b(:,1),Xmax,Ymax,R,C)));
        [bx_min,by_min] = IJtoXY(min(b(:,2),min(b(:,1),Xmax,Ymax,R,C)));
        diameter = abs(by_max-by_min);
        t_i = mean(b(:,2));
        t_j = mean(b(:,1));
        [tree_x,tree_y] = IJtoXY(t_i,t_j,Xmax,Ymax,R,C);
        %Determine row
        
        plot(b(:,2),b(:,1),'g','LineWidth',3);
    end
    
    
end


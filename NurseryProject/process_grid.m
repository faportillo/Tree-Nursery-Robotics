function nursery = process_grid(grid,nursery,K,Xmax,Ymax,R,C)
%PROCESS_GRID Processes Occupancy Grid
%   grid: Occupancy Grid
%   nursery: Nursery Object, use AddTree Function
    %Process noise from grid
    imshow(grid) %Show original image
    figure
    %Remove low probability values by converting to BW
    indices = find(abs(grid)<0.4);
    grid(indices) = 0;
    grid = double(im2bw(grid,0.4));
    grid = imresize(grid,[4000,4000]);
    grid = imresize(grid,[R,C]);

    imshow(grid)
    
    [B,L,N] = bwboundaries(grid,'noholes');
    n = length(B);
    colors=['b' 'g' 'r' 'c' 'm' 'y'];
    hold on
    
    th = 0:pi/50:2*pi; 
    
    tree_xs=[];
    tree_ys=[];
    tree_ds=[];
    for k=2:n
        b=B{k};
        
        im_d = abs(max(b(:,1))-min(b(:,1)));
        bc_x = abs(max(b(:,2))+min(b(:,2)))/2;
        bc_y = abs(max(b(:,1))+min(b(:,1)))/2;
        
        xunit = ((im_d/2)*(8/9))*cos(th)+bc_x;
        yunit = ((im_d/2)*(8/9))*sin(th)+bc_y;
        
        %Convert pixel to XY Points
        [bx_max,by_max] = IJtoXY(max(b(:,2)),max(b(:,1)),Xmax,Ymax,R,C);
        [bx_min,by_min] = IJtoXY(min(b(:,2)),min(b(:,1)),Xmax,Ymax,R,C);
       
        diameter = abs(by_max-by_min)*(8/9);
        %Filter out tiny boundary fragments
        if diameter < 0.2
           continue  
        end
        
        t_i = mean(b(:,1));
        t_j = mean(b(:,2));
        [im_d,diameter];
        [tree_x,tree_y] = IJtoXY(t_i,t_j,Xmax,Ymax,R,C);
        tree_xs = [tree_xs tree_x];
        tree_ys = [tree_ys tree_y];
        tree_ds = [tree_ds diameter];
        
        %Plot boundaries and numbers
        plot(xunit,yunit,'LineWidth',2);
        hold on
        rndRow = ceil(length(b)/(mod(rand*k,7)+1));
        cidx = mod(k,length(colors))+1;
        %plot(b(:,2),b(:,1),'g','LineWidth',3);
        col = b(rndRow,2); row = b(rndRow,1);
        h = text(col+1, row-1, num2str(L(row,col)));
        set(h,'Color',colors(cidx),'FontSize',14,'FontWeight','bold');
    end
    values = 1:K;
    tree_list = [tree_xs;tree_ys;tree_ds;discretize(tree_xs,K,values)]';
    figure
    plot(tree_list(:,1),tree_list(:,2),'*');
    for k = 1:length(tree_list)
       nursery.AddTree(tree_list(k,1),tree_list(k,2),tree_list(k,3),tree_list(k,4));
    end
end


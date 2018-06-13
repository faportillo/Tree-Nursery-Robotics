%% output measured diameters in the correct order from ProcessGrid
format long

% save treeList and treeListIndex
m = 1;
for r = 1:obj.nRows
  for tree = 1:obj.treesInRow(r)
    new_list(m,:) = obj.treeMeasurements{r}(tree, :);
    m = m+1;
  end
end

%% organize actual values to match measured values format

% measured values
x_m = new_list(:,1)';
y_m = new_list(:,2)';
d_m = new_list(:,3)';

% convert from pixels to meters and multiply by 2 for diam
d_a = 2.*(rad.*(Xmax/c)); 

% attach the row numbers of each tree to y and d
val_convert = [y_a' d_a' tree_list(:,4)]; 

% find indices of trees per row
ind1 = val_convert(:,3) == 1;
ind2 = val_convert(:,3) == 2;
ind3 = val_convert(:,3) == 3;
ind4 = val_convert(:,3) == 4;
ind5 = val_convert(:,3) == 5;
ind6 = val_convert(:,3) == 6;

% use logical indices to return sub-matrices containing trees grouped by
% row
t1 = val_convert(ind1,:);
t2 = val_convert(ind2,:);
t3 = val_convert(ind3,:);
t4 = val_convert(ind4,:);
t5 = val_convert(ind5,:);
t6 = val_convert(ind6,:);

% flip sub-matrices 
t1 = flipud(t1);
t2 = flipud(t2);
t3 = flipud(t3);
t4 = flipud(t4);
t5 = flipud(t5);
t6 = flipud(t6);

% store the newly sorted values
y_list = [t1(:,1);t2(:,1);t3(:,1);t4(:,1);t5(:,1);t6(:,1)];

d_a = [t1(:,2);t2(:,2);t3(:,2);t4(:,2);t5(:,2);t6(:,2)];
y_a = 50 - y_list;

% calculate the error
for i = 1:1:length(x_a)
    x_error(i) = abs((x_a(i) - x_m(i)));
    y_error(i) = abs((y_a(i) - y_m(i)));
    d_error(i) = abs((d_a(i) - d_m(i)));
end

% plot
figure(4)
hist(x_error)
title('x position error')
xlabel('bins')
ylabel('number of occurences')

figure(5)
hist(y_error)
title('y position error')
xlabel('bins')
ylabel('number of occurences')

figure(6)
hist(d_error)
title('diameter error')
xlabel('bins')
ylabel('number of occurences')

% display descriptive statitics 
xMean = mean(x_error)
xSD = std(x_error)
xRMS = rms(x_error)
xmin = min(x_error)
xmax = max(x_error)
xPer = prctile(x_error, 95)

yMean = mean(y_error)
ySD = std(y_error)
yRMS = rms(y_error)
ymin = min(y_error)
ymax = max(y_error)
yPer = prctile(y_error, 95)

dMean = mean(d_error)
dSD = std(d_error)
dRMS = rms(d_error)
dmin = min(d_error)
dmax = max(d_error)
dPer = prctile(d_error, 95)


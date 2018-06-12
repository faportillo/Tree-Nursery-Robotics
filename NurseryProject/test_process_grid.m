clear
close all
%load('ExampleScan2.mat')
load('ExampleScan3.mat')
r = size(prob_grid,1); c = size(prob_grid,2);
W = 3;
K = 5;
Xmax = W*(K-1)+40; Ymax = Xmax;

process_grid(prob_grid,[],K,Xmax,Ymax,r,c);

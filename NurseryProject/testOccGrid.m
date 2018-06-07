clear
close all

global occ_grid;
global bitmap;
global Pfree; global Pocc;

R = 5000; C = 5000; %numbers of rows and columns of bitmap
bitmap = zeros(R, C); %initialize as empty
map=zeros(R, C);
Xmax = 150; Ymax = 150;

%test rectangular obstacle
Xsw=70; Ysw = 30;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;
angleSpan = pi; angleStep = angleSpan/360; rangeMax = 200;

% initialize as empty
occ_grid = ones(size(bitmap));
Pocc = 0.7; Pfree = 1-Pocc;

for k=1:5
    Tl = se2([10+10*k  5 pi/2]);
    p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = medfilt1(p(i,2));
        n = updateLaserBeamGrid(angle, range, Tl, R, C, Xmax, Ymax);
    end
end

imagesc(bitmap);
colorbar

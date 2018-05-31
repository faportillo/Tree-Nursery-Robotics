clear all;
global bitmap;
global rangeMax;

R = 500; C = 500;
map=zeros(R, C);
grid = zeros(R, C) + 0.1;
bitmap = 0.0* ones(R, C); %initialize as empty
Xmax = 150; Ymax = 150;
%test rectangular obstacle
Xsw=70; Ysw = 30;
Xne=Xsw + 30; Yne= Ysw + 20;
[Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
[Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
map(Ine:Isw, Jsw:Jne) = 1;
angleSpan = pi; angleStep = angleSpan/360; rangeMax = 200;
for k=1:5
    Tl = se2([10+10*k  5 pi/2]);
    p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax);  
    for i=1:length(p)
        angle = p(i,1); range = p(i,2);
%        n = updateLaserBeamBitmap(angle, range, Tl, R, C, Xmax, Ymax);
        grid = UpdateLaserBeamGrid(grid, angle, range, Tl, R, C, Xmax, Ymax);
    end
end

imagesc(grid);
colorbar

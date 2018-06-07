%test laser scanner
close all; clear all; clear workspace;

% create robot workspace
R = 1000; % defines dimensions of occupancy grid RXR
C = 1000;

map=makemap(R); % make an occupancy map

Xmax = 150; Ymax = 150;
% Xmax = 1000; Ymax = 1000;

% map=zeros(R, C);
% % test rectangular obstacle
% Xsw=70; Ysw = 30;
% Xne=Xsw + 30; Yne= Ysw + 20;
% [Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
% [Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
% map(Ine:Isw, Jsw:Jne) = 1;

% define scanner parameters 
angleSpan=pi; % shoots rays from -angleSpan/2 to angleSpan/2
angleStep=pi/720;
rangeMax= 200;
% Tl=se2([75 10 pi/2]); %current laser pose (coordinate frame) wrt world
Tl=se2([0 0 pi/4]); %current laser pose (coordinate frame) wrt world

% call scanner function
p = laserScannerNoisy(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax); %p contains angle and range readings
p(:,2) = medfilt1(p(:,2));

% plot scanner output
axis equal
figure(2);
plot(p(:,1)*180/pi, p(:,2)); % plot angle and range - it is displayed in ccw, from -angleSpan/2 to +angleSpan/2
xlabel('angle (degrees)')
ylabel('range')


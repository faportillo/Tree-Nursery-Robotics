
%test laser scanner

addpath geom2d;
%clear all;
clear workspace;
close all
% create robot workspace
R = 1000; C = 1000;
map=makemap(R);

Xmax = 150; Ymax = 150;

% map=zeros(R, C);
% %test rectangular obstacle
% Xsw=70; Ysw = 30;
% Xne=Xsw + 30; Yne= Ysw + 20;
% [Isw, Jsw] = XYtoIJ(Xsw, Ysw, Xmax, Ymax, R, C);
% [Ine, Jne] = XYtoIJ(Xne, Yne, Xmax, Ymax, R, C);
% map(Ine:Isw, Jsw:Jne) = 1;

angleSpan=pi;angleStep=pi/720;
rangeMax=200;
Tl=se2([75 10 pi/2]); %current laser pose (coordinate frame) wrt world


p = laserScanner(angleSpan, angleStep, rangeMax, Tl, map, Xmax, Ymax); %p contains angle and range readings

figure(2);
plot(p(:,1)*180/pi, p(:,2)); % plot angle and range - it is displayed in ccw, from -angleSpan/2 to +angleSpan/2



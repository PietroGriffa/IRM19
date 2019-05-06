%% Lab07

clear all
close all

%% Print rectangula trajectory 
cal = 15.9;
[x,y] = importdata('RectangularCoord.txt');
figure(0);
title('MoveMotorRectangular');
plot(x*cal,y*cal,'ro-');
xlabel('x [mm]');
ylabel('y [mm]');
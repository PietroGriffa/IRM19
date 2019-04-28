%% Tabula rasa
clear all
close all

%% Lab 06

% Import and plot the trajectory tracked in tracking.cpp
coordinates = importdata('Coordinates.txt');
plot(coordinates(:,1),coordinates(:,2),'-r');
grid on
axis equal
title('Trajectory');
xlabel('x');
ylabel('y');
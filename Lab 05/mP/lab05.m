%% LAB 05

clear all
close all

%% Import data

FRN = importdata('FRN.txt');
FR = importdata('FR.txt');
FSN = importdata('FSN.txt');
FS = importdata('FS.txt');

time = FRN.data(2:end,1);

%% Noise
% We want to discover the frequency of the noise, to better get rid of it
% in the filtering process

ramp_noise = FRN.data(2:end,2)-FR.data(2:end,2);
sinus_noise = FSN.data(2:end,2)-FS.data(2:end,2);

% two ways to clculate the frequency: period between two peaks, or
% semi-period between two zeros
locZR = find(~ramp_noise,10,'last');    % ramp noise works better with zeros
locZS = find(~sinus_noise,10,'last');
[pkSN locPSN] = findpeaks(sinus_noise); % sinus noise works better with peaks
[pkRN locPRN] = findpeaks(ramp_noise);

freqNR = 0.5/(time(locZR(2))-time(locZR(1)));
% freqNS = 0.5/(time(locZS(2))-time(locZS(1)));
% freqNR = 1/(time(locPRN(2))-time(locPRN(1)));
freqNS = 1/(time(locPSN(2))-time(locPSN(1)));



%%%%%%%%%%%%%%%%%%%%
% got some problem with sinus frequancy: it's debugging time
Tsn = 1/freqNS;

%%%%%%%%%%%%%%%%%%%%

figure(99)
plot(time,ramp_noise,'g');
grid on;
title('Ramp Noise');
xlim([time(locZR(1)-1) time(locZR(5)+1)]);

figure(98)
plot(time,sinus_noise,'r');
grid on;
title('Sinus Noise');
xlim([time(locZS(1)-1) time(locZS(5)+1)]);

%% Plot data

im = figure(1);

plot(time,FRN.data(2:end,2),'b'); 
grid on
hold on
title('Ramp with noise');
xlabel('Sample');
ylabel('Magnitude');
plot(time,FRN.data(2:end,3),'g');
plot(time,FRN.data(2:end,4),'k'); 
legend('Raw data with noise','Moving Average','Blackman');

% c1 = uicontrol('Style','checkbox','String','Raw signal with noise');
% c1.Position = [20 60 60 20];
% c1.Callback = @plotButtonPushed;
% c2 = uicontrol('Style','checkbox','String','Moving average');
% c2.Position = [20 40 60 20];
% c3 = uicontrol('Style','checkbox','String','Blackman');


im = figure(2);

plot(time,FSN.data(2:end,2),'b'); 
grid on
hold on
title('Sinus');
xlabel('Sample');
ylabel('Magnitude');
ylim([-2,2]);
plot(time,FSN.data(2:end,3),'g');
plot(time,FSN.data(2:end,4),'k'); 
legend('Raw data with noise','Moving Average','Blackman');

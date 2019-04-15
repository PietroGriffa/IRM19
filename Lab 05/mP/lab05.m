%% LAB 05

clear all
close all

%% Import data

FRN = importdata('FRN.txt');
FR = importdata('FR.txt');
FSN = importdata('FSN.txt');
FS = importdata('FS.txt');
R = importdata('ramp.txt');
RN = importdata('ramp_noise.txt');
S = importdata('sinus.txt');
SN = importdata('sinus_noise.txt');

time = 1000*FRN.data(:,1);

%% Noise
% We want to discover the frequency of the noise, to better get rid of it
% in the filtering process

ramp_noise = FRN.data(:,2)-FR.data(:,2);
sinus_noise = FSN.data(:,2)-FS.data(:,2);

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

plot(time,FRN.data(:,2),'b'); 
grid on
hold on
title('Ramp with noise');
xlabel('Sample');
ylabel('Magnitude');
plot(time,FR.data(:,2),'r');
plot(time,FRN.data(:,3),'g');
plot(time,FRN.data(:,4),'k'); 
legend('Raw data with noise','Raw data without noise','Moving Average [N=50]','Blackman [M=200,fc=0.012]');

% c1 = uicontrol('Style','checkbox','String','Raw signal with noise');
% c1.Position = [20 60 60 20];
% c1.Callback = @plotButtonPushed;
% c2 = uicontrol('Style','checkbox','String','Moving average');
% c2.Position = [20 40 60 20];
% c3 = uicontrol('Style','checkbox','String','Blackman');


im = figure(2);

plot(time,FSN.data(:,2),'b'); 
grid on
hold on
title('Sinus');
xlabel('Sample');
ylabel('Magnitude');
ylim([-2,2]);
plot(time,FS.data(:,2),'r');
plot(time,FSN.data(:,3),'g');
plot(time,FSN.data(:,4),'k'); 
legend('Raw data with noise','Raw data without noise','Moving Average [N=50]','Blackman [M=200,fc=0.012]');

%%
im = figure(3);

plot(time,FS.data(:,2),'b'); 
grid on
hold on
title('Sinus; effect on undistorted signal');
xlabel('Sample');
ylabel('Magnitude');
ylim([-2,2]);
plot(time,FS.data(:,3),'g');
plot(time,FS.data(:,4),'k'); 
legend('Raw data without noise','Moving Average [N=50]','Blackman [M=200,fc=0.012]');

im = figure(4);

plot(time,FR.data(:,2),'b'); 
grid on
hold on
title('Ramp; effect on undistorted signal');
xlabel('Sample');
ylabel('Magnitude');
plot(time,FR.data(:,3),'g');
plot(time,FR.data(:,4),'k'); 
legend('Raw data without noise','Moving Average [N=50]','Blackman [M=200,fc=0.012]');
%%

figure(3);

plot(R(:,1),R(:,2),'b'); 
grid on
hold on
title('Ramp');
xlabel('Sample');
ylabel('Magnitude');
plot(RN(:,1),RN(:,2),'k'); 

figure(4);

plot(S(:,1),S(:,2),'b'); 
grid on
hold on
title('Sinus');
xlabel('Sample');
ylabel('Magnitude');
plot(SN(:,1),SN(:,2),'k'); 

figure(5);

plot(R(:,1),R(:,2)-RN(:,2),'b'); 
grid on
hold on
title('Ramp Noise');
xlabel('Sample');
ylabel('Magnitude'); 

figure(6);

plot(S(:,1),S(:,2)-SN(:,2),'b'); 
grid on
hold on
title('Sinus Noise');
xlabel('Sample');
ylabel('Magnitude');


%%

Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 6834;             % Length of signal
t = (0:L-1)*T;        % Time vector

YS = fft(SN(:,2)-S(:,2));
PS2 = abs(YS/L);
PS1 = PS2(1:L/2+1);
PS1(2:end-1) = 2*PS1(2:end-1);

YR = fft(RN(:,2)-R(:,2));
PR2 = abs(YR/L);
PR1 = PR2(1:L/2+1);
PR1(2:end-1) = 2*PR1(2:end-1);

f = Fs*(0:(L/2))/L;
%%
figure(7)
plot(f,PS1) 
grid on
title('Single-Sided Amplitude Spectrum of Noise in Sinus')
xlabel('f (Hz)')
ylabel('|P1(f)|')

figure(8)
plot(f,PR1)
grid on
title('Single-Sided Amplitude Spectrum of Noise in Ramp')
xlabel('f (Hz)')
ylabel('|P1(f)|')
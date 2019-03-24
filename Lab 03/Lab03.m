clc; close all; clear all;
%% 1st order low pass:
f = 100;
f2 = 85;
wc = 2*pi*f;
wc2 = 2*pi*f2;


s= tf('s');
C = wc/(wc + s);
C2 = wc2/(wc2 + s);

options = bodeoptions;
options.FreqUnits = 'Hz'; % or 'rad/second', 'rpm', etc.
figure(1)
bode(C,options);
grid on

figure(2)
impulse(C);
grid on

figure(3)
step(C);
grid on

%%
mag = abs(wc/(wc+j*wc))
magdb = 20*log10(mag)
phase = angle(wc/(wc+j*wc));
phased = radtodeg(phase);
%%
fs = [20 40 60 80 100 200];
ws = 2*pi*fs;
p2p = [2.4 2.16 2 1.76 1.52 0.96]/2.4;
p2pdb = 20*log10(p2p);
ps = [20 34 40 42 48 69]*-1;

figure(4)
subplot(2,1,1)
plot(fs,p2pdb,'r-o')
title('Sampled magnitude response analog RC-filter')
xlabel('Hz [1/s]')
ylabel('Magnitude [dB]')
grid on
subplot(2,1,2)
plot(fs,ps,'r-o')
title('Sampled phase response analog RC-filter')
xlabel('Hz [1/s]')
ylabel('Phase shift [deg]')
grid on
%%

flin = linspace(10,210,201);
C2 = wc2./(wc2 + 1i*flin*2*pi);
C1 = wc./(wc + 1i*flin*2*pi);
%%

figure(5)

subplot(2,1,1)
plot(fs,p2pdb,'r.')
grid on
hold on
plot(flin,20*log10(abs(C2)),'b')
plot(flin,20*log10(abs(C1)),'k')
title('Magnitude comparison between analog RC-filter and calculated low-pass filters')
xlabel('Hz [1/s]')
ylabel('Magnitude [dB]')
legend('sampled data','analytic 1^{st} order lowpass f_c=85Hz','analytic 1^{st} order lowpass f_c=100Hz')

subplot(2,1,2)
plot(fs,ps,'r.')
grid on
hold on
plot(flin,rad2deg(angle(C2)),'b')
plot(flin,rad2deg(angle(C1)),'k')
title('Phase comparison between analog RC-filter and calculated low-pass filters')
xlabel('Hz [1/s]')
ylabel('Phase shift [deg]')
legend('sampled data','analytic 1^{st} order lowpass f_c=85Hz','analytic 1^{st} order lowpass f_c=100Hz')


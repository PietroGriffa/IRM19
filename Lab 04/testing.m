%% testing a Blackman window

% Selfmade
i = linspace(0,100);
m = linspace(0,100);
M = 100-1;
w =@(i) 0.42-0.5*cos(2*pi*i./M)+0.08*cos(4*pi*i./M);

%% plotting the Blackman window

% the Blackman window on length N creates a function centered in N/2
plot(m,w(i));
grid on

% mathworks function
figure(2);
plot(blackman(100));    % the parameter is the length of the window

%% filter kernel

h =@(i) w(i)*(sin(2*pi*fc(i-M*0.5)))/(i-M*0.5);

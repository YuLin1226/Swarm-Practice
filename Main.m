clear;
clc;

%% Parameters
time_interval = [0 50];         % �p��ɶ�
ICs = [0,0,5,0]';                   % 4�Ӫ�l����: x, dx, y, dy
circle = [5, 5, 3];


%% ODE45
[t,Robot_1] = ode45(@MobileRobotDynamics, time_interval, ICs);
[t,Robot_2] = ode45(@MobileRobotDynamics, time_interval, [7, 0, 7, 0]);

%% Result Plotting
PlotCircle(circle(1), circle(2), circle(3));
axis([0, 10, 0, 10]);
grid on; hold on;
for i = 1 : length(Robot_1)
plot(Robot_1(i,1), Robot_1(i,3), 'b.');
plot(Robot_2(i,1), Robot_2(i,3), 'b.');
pause(0.01);
end
clear;
clc;

%% Parameters
time_interval = linspace(0,4000,11);              % 計算時間
IC_1 = [-3, 0, 3, 0]';                  % 4個初始條件: x, dx, y, dy
IC_2 = [1, 0, 1, 0]';                   % 4個初始條件: x, dx, y, dy
IC_3 = [5, 0, 2, 0]';                   % 4個初始條件: x, dx, y, dy
IC_4 = [5, 0, 5, 0]';                   % 4個初始條件: x, dx, y, dy

State_1 = [];                           % 存state 資料
State_2 = [];                           % 存state 資料
State_3 = [];                           % 存state 資料
State_4 = [];                           % 存state 資料
%% ODE45
for i = 1 : length(time_interval)-1
    time = [time_interval(i) , time_interval(i+1)];

    [t,Robot_1] = ode45(@RobotDynamics, time, IC_1, [], [IC_2(1), IC_3(1), IC_4(1)], [IC_2(3), IC_3(3), IC_4(3)]);
    [t,Robot_2] = ode45(@RobotDynamics, time, IC_2, [], [IC_1(1), IC_3(1), IC_4(1)], [IC_1(3), IC_3(3), IC_4(3)]);
    [t,Robot_3] = ode45(@RobotDynamics, time, IC_3, [], [IC_1(1), IC_2(1), IC_4(1)], [IC_1(3), IC_2(3), IC_4(3)]);
    [t,Robot_4] = ode45(@RobotDynamics, time, IC_4, [], [IC_1(1), IC_2(1), IC_3(1)], [IC_1(3), IC_2(3), IC_3(3)]); 
    
    State_1 = [State_1 ; Robot_1];
    State_2 = [State_2 ; Robot_2];
    State_3 = [State_3 ; Robot_3];
    State_4 = [State_4 ; Robot_4];
    
    IC_1 = Robot_1(end, :);
    IC_2 = Robot_2(end, :);
    IC_3 = Robot_3(end, :);
    IC_4 = Robot_4(end, :);
    
end
%% Result Plotting
PlotCircle(0, 0, 3);
axis([-6, 6, -6, 6]);
grid on; hold on;

n = 1;
k = 0.5;

pos = [ State_2(end,1), State_2(end,3);
        State_4(end,1), State_4(end,3);
        State_1(end,1), State_1(end,3);
        State_3(end,1), State_3(end,3)];
   
plot( [pos(4,1),pos(1,1)] , [pos(4,2),pos(1,2)] , 'k--' );    
for i = 1:3
    plot( [pos(i,1),pos(i+1,1)] , [pos(i,2),pos(i+1,2)] , 'k--' );
end

% for i = fix(length(State_1)*k) : length(State_1)
%     i
%     plot(State_1(n*i,1), State_1(n*i,3), 'b.');
%     plot(State_2(n*i,1), State_2(n*i,3), 'r.');
%     plot(State_3(n*i,1), State_3(n*i,3), 'g.');
%     plot(State_4(n*i,1), State_4(n*i,3), 'm.');
%     pause(0.01);
% end

function dq = RobotDynamics(t, q, x, y)

M = 1;
B = 1;
kd = 9;
kr = 0.15;
Q = 10;
ksk = 1;
qt = [0, 0];    % Virtual circle
R = 3;


r = ( (q(1)- x).^2 + (q(3) - y).^2).^0.5;         % Other Robots
cosine = ( q(1) - x ) ./ r;
sine = ( q(3) - y ) ./ r;

F  = [ -ksk * ( q(1) - qt(1) ) * ( (q(1)-qt(1))^2 + (q(3)-qt(2))^2 - R^2 ) ;
       -ksk * ( q(3) - qt(2) ) * ( (q(1)-qt(1))^2 + (q(3)-qt(2))^2 - R^2 )];

F_x = kr*Q*Q*sum( cosine ./ (r.^2) );   
F_y = kr*Q*Q*sum( sine ./ (r.^2) ); 

dq = [ q(2);
       F(1)/M - kd/M*q(2) - B/M*q(2) + F_x;
       q(4);
       F(2)/M - kd/M*q(4) - B/M*q(4) + F_y];
end
clear;
clc;

%% Parameters


time_interval = linspace(0,4000,11);              % 計算時間
IC_1 = [-3, 0, 3, 0]';                  % 4個初始條件: x, dx, y, dy
IC_2 = [1, 0, 1, 0]';                   % 4個初始條件: x, dx, y, dy
IC_3 = [5, 0, 2, 0]';                   % 4個初始條件: x, dx, y, dy
IC_4 = [5, 0, 5, 0]';                   % 4個初始條件: x, dx, y, dy

State_vs = [];
State_1 = [];                           % 存state 資料
State_2 = [];                           % 存state 資料
State_3 = [];                           % 存state 資料
State_4 = [];                           % 存state 資料
%% ODE45
for i = 1 : length(time_interval)-1
    time = [time_interval(i) , time_interval(i+1)];
    vs = [  0 + time(1)/400;                 % Virtual Circle
            0 + sin(time(1)/400);
            3];              
    State_vs = [State_vs,vs];
    [t,Robot_1] = ode45(@RobotDynamics, time, IC_1, [], [IC_2(1), IC_3(1), IC_4(1)], [IC_2(3), IC_3(3), IC_4(3)], vs);
    [t,Robot_2] = ode45(@RobotDynamics, time, IC_2, [], [IC_1(1), IC_3(1), IC_4(1)], [IC_1(3), IC_3(3), IC_4(3)], vs);
    [t,Robot_3] = ode45(@RobotDynamics, time, IC_3, [], [IC_1(1), IC_2(1), IC_4(1)], [IC_1(3), IC_2(3), IC_4(3)], vs);
    [t,Robot_4] = ode45(@RobotDynamics, time, IC_4, [], [IC_1(1), IC_2(1), IC_3(1)], [IC_1(3), IC_2(3), IC_3(3)], vs); 
    
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
% -------------Virtual Circle Plot-------------
for j = 1:length(State_vs)
    PlotCircle(State_vs(1,j), State_vs(2,j), State_vs(3,j));
    axis([-10, 20, -10, 20]);
    grid on; hold on;
end
% -------------Virtual Circle Plot-------------


% -------------Robots Plot-------------
n = fix(length(State_4(:,1))/200);
for i = 1:200
    r1=plot(State_1(n*i,1), State_1(n*i,3), 'bo');
    r2=plot(State_2(n*i,1), State_2(n*i,3), 'ro');
    r3=plot(State_3(n*i,1), State_3(n*i,3), 'go');
    r4=plot(State_4(n*i,1), State_4(n*i,3), 'mo');
    pause(0.01);
    if i < 200
        delete(r1);
        delete(r2);
        delete(r3);
        delete(r4);
    end
end
% -------------Robots Plot-------------

function dq = RobotDynamics(t, q, x, y, vs)

% ----------系統方程式參數設定----------
M = 1;
B = 1;
kd = 9;
kr = 0.15;
Q = 10;
ksk = 1;
% ----------系統方程式參數設定----------

qt = [vs(1), vs(2)];    % Virtual circle
R = vs(3);


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
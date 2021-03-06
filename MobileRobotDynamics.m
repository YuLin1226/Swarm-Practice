function dq = MobileRobotDynamics(t, q, circle)
M = 1;
B = 1;
kd = 9;
ksk = 1;

qt = [circle(1), circle(2)];
R = circle(3);

f  = [ -ksk * ( q(1) - qt(1) ) * ( (q(1)-qt(1))^2 + (q(3)-qt(2))^2 - R^2 ) ;
       -ksk * ( q(3) - qt(2) ) * ( (q(1)-qt(1))^2 + (q(3)-qt(2))^2 - R^2 )];

dq = [ q(2);
       f(1)/M - kd/M*q(2) - B/M*q(2);
       q(4);
       f(2)/M - kd/M*q(4) - B/M*q(4)];
end
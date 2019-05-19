










%x = [theta_dot, theta_dot, theta, theta]
function [dx] = two_link_dynamics(t, x, torque, robot)
theta_1_dot = x(1);
theta_2_dot = x(2);
theta_1 = x(3);
theta_2 = x(4);
gravity = [0; 0];
m1 = robot.m1;
m2 = robot.m2;
l1 = robot.l1;
r2 = robot.r2;
r1 = robot.r1;
I1 = robot.I1;
I2 = robot.I2;
alpha = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
beta = m2*l1*r2;
delta = I2 + m2*r2^2;
theta = [0;0];
mass_matrix = [alpha + 2*beta*cos(theta(2)), delta + beta*cos(theta(2));
               delta + beta*cos(theta(2)) , delta];
           
coriolis_matrix = [-beta*sin(theta(2))*x(2), -beta*sin(theta(2))*(x(1) + x(2));
                    beta*sin(theta(2))*x(1), 0];
                
dx = [0;0;0;0];
dx(1:2) = inv(mass_matrix) * (-coriolis_matrix*[theta_1_dot; theta_2_dot] - gravity + torque);
dx(3) = theta_1_dot;
dx(4) = theta_2_dot;
dx          
  
end
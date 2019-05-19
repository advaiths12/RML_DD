%include the dynamics of a 2 link robot, 
%add a mass and gravity and check the deflection
%use ODE45 for the dynamics of the 2 link arm 

tspan = [0 5];
tspan_slice = linspace(0, 5, 1000);
torque = [0; 0];
m1 = .5;
m2 = .5;
l1 = 40/1000;
r2 = 50/1000;
r1 = 50/1000;
I1 = .5;
I2 = .5;
l2 = 50/1000;
l3 = l2;
l4 = l2;
robot_arm = FiveBarLinkage(l1,l2,l3,l4, m1, m2,r2, r1, I1, I2);
P = .125;
I = .00005;
D = .7;
set_point = [-.02, .08]; %meters in workspace
new_thetas_veloc = [0,0];
curr_theta = [1.5,1.5];
I_error_min = -1;
I_error_max = 1;
time = 0;
figure();
hold on;
axis([-1, 1, -1, 1]);
err = [0,0];
pause on;
I_error = [0,0];
prev_step = 0;
dq1 = 1.5; %get values from ode45
dq2 = 1.5;
q1 = 0;
q2 = 0;
for i = tspan_slice(2:end)
    pause(.01);
    [t, y] = ode45(@(t, x) robot_arm.two_link_dynamics(t, x, torque),[prev_step, i], [dq1; dq2; q1; q2]);
    prev_step = i; %update previous step
    dq1 = y(end, 1); %get values from ode45
    dq2 = y(end, 2);
    q1 = y(end, 3);
    q2 = y(end, 4);
    curr_point = robot_arm.fk([q1; q2]);
    err = .001*(set_point - curr_point);
    force = err';
    torque = robot_arm.jacobian([q1; q2])'*force; 
    plot(curr_point(1), curr_point(2), "g+");
    plot(set_point(1), set_point(2), "r.");
end
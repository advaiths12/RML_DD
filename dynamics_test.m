tspan = [0 5];
torque = [5; 5];
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
[t, y] = ode45(@(t, x) robot_arm.two_link_dynamics(t, x, torque),tspan, [5; 5; 0; 0]);
figure();
title('Y-Position vs. Time');
plot(t, y(:,1), '-o');
axis tight;
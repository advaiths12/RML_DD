clear all
params.masses = [0.1;0.1;0.1;0.1];
params.lengths = [0.1;0.1;0.1;0.1;0];
params.g = 9.81
y0 = [3*pi/4;2*pi-pi/2;pi/4;pi/2;0;0;0;0];
setpoint = y0(1:4) + [1; .5; .5; .5]
freq = 1000;
ts = 1/freq;
tspan = 0:ts:2
tau = [0.0;0;0.0;0];
kp = 10;
kd = 1;
Q = [];
robot_arm = FiveBarLinkage(0.1,0.1,0.1,0.1, 0.1, 0.1,0.1, 0.1, .5, .5);
for t = tspan
  [T,Y] = ode45(@(t,y) odefun(t,y,tau,params),[t t+ts], y0);
  y0 = Y(end,:);
  Q = [Q y0'];
  fk1 = robot_arm.fk([y0(1); y0(5)]);
  f = .0001*([setpoint(1), setpoint(4)] - [y0(1), y0(4)]);
  temp = robot_arm.jacobian(y0(3:4))'*f';
  tau = [temp(1); 0; temp(2); 0];
end

figure()
fk = Five_Bar_FK(0, setpoint, params.masses, params.lengths, params.g)
fk(1,:)
l = plot(fk(1,:),fk(2,:))
xlim(0.2*[-1 1])
ylim(0.2*[-1 1])
for i = 1:size(Q,2)
  y = Q(:,i);
  % fk = Five_Bar_FK(0, y, params.masses, params.lengths, params.g);
  P1 = [0;0];
  P2 = [params.lengths(1)*cos(y(1));params.lengths(1)*sin(y(1))];
  P3 = P2 + [params.lengths(2)*cos(y(1) + y(2));params.lengths(2)*sin(y(1) + y(2))];
  P5 = [params.lengths(5);0];
  P4 = P5 + [params.lengths(3)*cos(y(3));params.lengths(3)*sin(y(3))];
  fk = [P1 P2 P3 P4 P5];
  l.XData = fk(1,:);
  l.YData = fk(2,:);
  drawnow;
  pause(0.01)
end



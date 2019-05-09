addpath(genpath('Screws'));
addpath(genpath('fcn_support'));
syms q1 q2 q4 q5 real
syms dq1 dq2 dq4 dq5 real
syms ddq1 ddq2 ddq4 ddq5 real
syms l1 l2 l3 l4 l5 positive
syms m1 m2 m3 m4 positive
syms t g real
syms tau1 tau2 tau4 tau5 real
% Generalized positions
q = [q1;q2;q5;q4];
% Generalized velocities
dq = [dq1;dq2;dq5;dq4];
% generalized Accelerations
ddq = [ddq1;ddq2;ddq5;ddq4];
% generalized forces
tau = [tau1 tau2 tau5 tau4]';

% Position of center of mass:
p1 = [l1*cos(q1);l1*sin(q1)];
p2 = [p1(1)+l2*cos(q1+q2) ; p1(2)+l2*sin(q1+q2)];

p4= [l4*cos(q5) + l5; l4*sin(q5)];
p3 = [l4*cos(q5) + l5 + l3*cos(q5+q4); l4*sin(q5) + l3*sin(q5+q4)];

% Velocity of center of mass:
v1 = get_vel(p1, q, dq)
v2 = get_vel(p2,q,dq)
v3 = get_vel(p3,q,dq)
v4 = get_vel(p4,q,dq)


% Kinetic Energy:
T1 = m1*v1'*v1/2;
T2 = m2*v2'*v2/2;
T3 = m3*v3'*v3/2;
T4 = m4*v4'*v4/2;
T = T1 + T2 + T3 + T4;

% Potential Energy:
V1 = m1*g*p1(2);
V2 = m2*g*p2(2);
V3 = m3*g*p3(2);
V4 = m4*g*p4(2);
V = V1 + V2 + V3 + V4;

[M,C,N] = get_mat(T,V,q,dq)
M = simplify(M)
C = simplify(C)
N = simplify(N)


% Figure out the constraints
a = [l1*cos(q1) + l2*cos(q1+q2) - l4*cos(q5) - l3*cos(q5+q4);
     l1*sin(q1) + l2*sin(q1+q2) - l4*sin(q5) - l3*sin(q5+q4);]

A = get_vel(a, q,dq)
A = jacobian(A,dq)

A_dot = [get_vel(A(1,:),q,dq) get_vel(A(2,:),q,dq)]'


% Compute the constraint system
M_bar = [M A'; A zeros(2)]
D_bar = [tau-N-C*dq;-A_dot*dq]


% Write the functions to file
matlabFunction( M_bar, ...
      'file','Five_Bar_Mass_Matrix', ...
      'vars', {t, [q;dq], tau,[m1;m2;m3;m4], [l1;l2;l3;l4;l5],g})
matlabFunction( D_bar,...
      'file','Five_Bar_CN_Matrix', ...
      'vars', {t, [q;dq], tau, [m1;m2;m3;m4], [l1;l2;l3;l4;l5],g})

%%Compute the new constrained system (add the obstacle)
%Define the distance function
syms x0 y0 x1 y1 x2 y2 real;
distance = abs((y2-y1)*x0 - (x2-x1)*y0 +x2*y1 - x1*y2)/norm([x1;y1]-[x2;y2])
a_obstacle = subs(distance,[x1;y1;x2;y2],[p4;p3])

a_new = [a;a_obstacle]

A_new = simplify(get_vel(a_new, q, dq))
A_new = simplify(jacobian(A_new,dq))

A_dot_new = [get_vel(A_new(1,:),q,dq) get_vel(A_new(2,:),q,dq) get_vel(A_new(3,:),q,dq)]
size(A_dot_new)



%% Compute the reduced system 

y = [q1;q5]
dy = [dq1;dq5]
tau = [tau1;tau5]
Y = jacobian(y,q)


H = [A;Y]\[0;0;1;1]
H_dot = get_vel(H,q,dq)

M_t = H'*M*H;
C_t = H'*M*H_dot + H'*C*H;
N_t = H'*N;



matlabFunction( M_t, ...
      'file','Five_Bar_Mass_Matrix_Reduced', ...
      'vars', {t, [y;dy], tau,[m1;m2;m3;m4], [l1;l2;l3;l4;l5],g})
matlabFunction( D_bar,...
      'file','Five_Bar_CN_Matrix', ...
      'vars', {t, [y;dy], tau, [m1;m2;m3;m4], [l1;l2;l3;l4;l5],g})

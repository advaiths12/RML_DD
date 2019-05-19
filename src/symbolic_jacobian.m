syms a1 a2 a3 a4 a5 real
syms t1 t5 real

P2 = [a1*cos(t1);a1*sin(t1)];
P4 = [a4*cos(t5);a4*sin(t5)];

p2_minus_ph = (a2^2 - a3^2 + norm(P4-P2)^2)/(2*norm(P4-P2));



p3_minus_ph = sqrt(a2^2-p2_minus_ph^2);

Ph = P2 + (p2_minus_ph/norm(P2-P4))*(P4-P2);

x3 = Ph(1) - (p3_minus_ph/norm(P2-P4))*(P4(2)-P2(2));
y3 = Ph(2) + (p3_minus_ph/norm(P2-P4))*(P4(1)-P2(1));

simplify(x3)
simplify(y3)

thetas = [t1;t5];

J = jacobian([x3;y3],thetas)

matlabFunction(J, 'file', 'jacobian')

linkage = FiveBarLinkage(50/1000,20/1000,30/1000,45/1000);

point = linkage.fk([pi/4 3*pi/4])

J = linkage.jacobian([pi/4 3*pi/4])

pi/4
3*pi/4

x = [];
y = [];
for theta1 = 0:.01:pi
    for theta2 = 0:.01:pi
        point = linkage.fk([theta1, theta2]);
        x = [x, point(1)];
        y = [y, point(2)];
    end
end




syms a_1 a_2 a_3 a_4 a_5 real 

syms t_1 t_5 real

P2 = [a_1*cos(t_1); a_1*sin(t_1)];
P4 = [a_4*cos(t_5) - a_5; a_4*sin(t_5)];

p2_minus_ph = (a_3^2 - a_2^2 + norm(P4-P2)^2)/(2*(norm(P4-P2)));

Ph = P2 + (p2_minus_ph/norm(P2-P4))*(P4-P2);

p3_minus_ph = sqrt(a_2^2 - p2_minus_ph^2);

x3 = Ph(1) + (p3_minus_ph/norm(P2-P4))*(P4(2)-P2(2));
y3 = Ph(2) - (p3_minus_ph/norm(P2-P4))*(P4(1)-P2(1));

P3 = [x3;y3];
pretty(simplify(P3))
matlabFunction(P3, 'File', 'fk');

T = [t_1;t_5];

J = simplify(jacobian(P3, T))
matlabFunction(J, 'File', 'jacobian');


jacobian_c(1,1,1,1,0,pi/4,3*pi/4)



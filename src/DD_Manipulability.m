linkage = FiveBarLinkage(50/1000,20/1000,30/1000,45/1000);

x = [];
y = [];
accum_v_manip = [];
accum_f_manip = [];

%test function for producing manipulability vectors
vectors = linkage.v_manip_axes([0, pi/2]);


figure();
plotv(vectors, '-');
title("Velocity Manipulability vectors");
%iterate through possible thetas
for theta1 = 0:.05:pi
    for theta2 = 0:.05:pi
        J = linkage.jacobian([theta1, theta2]);
        accum_v_manip = [accum_v_manip, det(J*J')];
        point = linkage.fk([theta1, theta2]);
        x = [x, point(1)];
        y = [y, point(2)];
        
    end
end


figure();
hold on;   
plot3(x, y, accum_v_manip, "ro", "LineWidth",1);
xlabel("x-coordinate (m)");
ylabel("y-coordinate (m)");
title("DD Velocity Manipulability in Workspace Coordinates");
%create robot class of measurements given on spec sheet
linkage = FiveBarLinkage(45/1000,30/1000,20/1000,50/1000);

x = [];
y = [];

%vectors for singularity fpositions
s_x = [];
s_y = [];
epsilon = 1e-5;
p_1 = [0, 0];
%iterate through possible thetas
figure();
%%TODO choose a specific space and do finite differences, compare to jacobian
for theta1 = 0:.01:pi
    for theta2 = theta1:.01:theta1 + pi/2
         %determined singularity position
            t2 = theta2;
            point = linkage.fk([theta1, theta2]);
            p_3 = point
%             if(abs(theta2 - theta1) == 1) 
%                         s_x = [s_x, p_3(1)];
%                         s_y = [s_y, p_3(2)];
%             end
            if(point(2) >= 0)
                jacob = jacobian(linkage.l1, linkage.l2, linkage.l3, linkage.l4, theta1, theta2);
                
                %get points of arm
                p_2 = [linkage.l1*cos(theta1), linkage.l1*sin(theta1)];
                p_4 = [linkage.l4*cos(t2), linkage.l4*sin(t2)];
                %check boundaries
                norm_a_1 = norm(p_2 - p_1);
                norm_a_2 = norm(p_3 - p_2);
                norm_a_3 = norm(p_3 - p_4);
                norm_a_4 = norm(p_4 - p_1);
                
               if(abs(norm_a_1 - linkage.l1) <= .1  && abs(norm_a_2 - linkage.l2) <= .1 && abs(norm_a_3 - linkage.l3) <= .1 && abs(norm_a_4 - linkage.l4) <= .1)
                  
                    if(abs(det(jacob)) <= epsilon)
                       
                     
%                 
                        %graph a1
                        hold on;
                        %plot a1
                        plot([p_2(1), p_1(1)], [p_2(2), p_1(2)], "r");

                        %plot a2
                        plot([p_3(1), p_2(1)], [p_3(2), p_2(2)], "b");

                        %plot a3
                        plot([p_3(1), p_4(1)], [p_3(2), p_4(2)], "b");

                        %plot a4
                        plot([p_4(1), p_1(1)], [p_4(2), p_1(2)], "g");
                        plot([p_3(1)], [p_3(2)], "rx");
                        axis([-.09 .09 -.09 .09]);
                        drawnow;
                        hold off;
                        

                        s_x = [s_x, point(1)];
                        s_y = [s_y, point(2)];
                        rad2deg(theta1 - theta2);
                    end

                    x = [x, point(1)];
                    y = [y, point(2)];

                end
            
           
        end
    end
end

%singularity_vals = DD_Hand_Singularity_Positions();
%s_x = [s_x;singularity_vals(:,1)];
%s_y = [s_y;singularity_vals(:,2)];

%plot
figure();
hold on;   
plot(x, y, "g", "LineWidth",1);
plot(s_x, s_y, "rx", "LineWidth", 1);
%axis([-.15 .15 0 .15]);
xlabel("x-coordinate (m)");
ylabel("y-coordinate (m)");
title("DD Hand Workspace");



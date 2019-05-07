classdef FiveBarLinkage
    properties
        l1
        l2
        l3
        l4
        m1
        m2
        r2
        r1
        I1
        I2
    end
    methods
      % Chain l1->l3->l4->l2->l1
        function obj = FiveBarLinkage(l1,l2,l3,l4, m1, m2, r2, r1, I1, I2)
            if (nargin==10)
                obj.l1 = l1;
                obj.l2 = l2;
                obj.l3 = l3;
                obj.l4 = l4;
                obj.m1 = m1;
                obj.m2 = m2;
                obj.r2 = r2;
                obj.r1 = r1;
                obj.I1 = I1;
                obj.I2 = I2;
            end
        end
        
        function theta = ik(obj, x, theta_3)
            theta_1 = acos((x - obj.l3*cos(theta_3))/obj.l1);
            y = obj.l1*sin(theta_1) + obj.l3*sin(theta_3);
            theta = obj.ik_from_point([x;y]);
        end
        
        function theta = ik_from_point(obj, point)
            x = point(1);
            y = point(2);
            Y = sqrt(power(x,2) + power(y,2));
            b = acos((power(Y,2) + power(obj.l1,2) - power(obj.l3,2))/(2*Y*obj.l1));
            a = acos((power(Y,2) + power(obj.l2,2) - power(obj.l4,2))/(2*Y*obj.l2));
            th = atan2(y,x);
            theta = [th-b;th+a];
        end
        %x = [theta_dot, theta_dot, theta, theta]
        function [dx] = two_link_dynamics(obj, t, x, torque)
        theta_1_dot = x(1);
        theta_2_dot = x(2);
        theta_1 = x(3);
        theta_2 = x(4);
        gravity = [0; 0];
        m1 = obj.m1;
        m2 = obj.m2;
        l1 = obj.l1;
        r2 = obj.r2;
        r1 = obj.r1;
        I1 = obj.I1;
        I2 = obj.I2;
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
        function point = fk(obj, theta)
            t_1 = theta(1);
            t_5 = theta(2);
            P_2 = [obj.l1*cos(t_1), obj.l1*sin(t_1)];
            P_4 = [obj.l4*cos(t_5), obj.l4*sin(t_5)];
            norm_P_2_minus_P_H = (obj.l2^2 - obj.l3^2 + norm(P_4 - P_2)^2)/ ... 
                                   (2*norm(P_4 - P_2));

            P_H = P_2 + norm_P_2_minus_P_H / norm(P_2 - P_4) * (P_4 - P_2);
            norm_P_3_minus_P_H = sqrt(obj.l2^2 - norm_P_2_minus_P_H^2);
        
            ee_x = P_H(1) + norm_P_3_minus_P_H / norm(P_2 - P_4) * (P_4(2) - P_2(2));
            ee_y = P_H(2) - norm_P_2_minus_P_H / norm(P_2 - P_4) * (P_4(1) - P_2(1));
            point = [ee_x, ee_y];
        end


        function theta_4 = theta_4(obj,theta)
          A = obj.l2*sin(theta(2)) - obj.l1*sin(theta(1));
          B = obj.l2*cos(theta(2)) - obj.l1*cos(theta(1));
          C = (power(obj.l3,2) - power(obj.l1,2) - power(obj.l2,2)...
          - power(obj.l4,2) + 2*obj.l1*obj.l2*cos(theta(2)-theta(1)))...
                  /(2*obj.l4);
          D = sqrt(power(A,2) + power(B,2));
          theta_4 = asin(C/D) - atan2(B,A);
        end


        function J = jacobian(obj, theta)
%           theta_4 = obj.theta_4(theta);
%           J = zeros(2);
%           J(1,1) = -obj.l2*sin(theta(2));
%           J(2,1) = +obj.l2*cos(theta(2));
%           J(1,2) = -obj.l4*sin(theta_4);
%           J(2,2) = obj.l2*sin(theta(2));

          P2 = [obj.l1*cos(theta(1)); obj.l1*sin(theta(1))];
          P4 = [obj.l4*cos(theta(2)); obj.l4*sin(theta(2))];
          d = norm(P2-P4);
          b = (obj.l2^2 - obj.l3^2 +norm(P4-P2)^2)/(2*norm(P4-P2));
          h = sqrt(obj.l2^2 - b^2);
          
          d1_x2 = obj.l1*sin(theta(1));
          d1_y2 = obj.l1*cos(theta(1));
          d5_x4 = obj.l4*sin(theta(2));
          d5_y4 = obj.l4*cos(theta(2));
          d1_y4 = 0;
          d1_x4 = 0;
          d5_x2 = 0;
          d5_y2 = 0;
          d1_d = ((P4(1) - P2(1))*(d1_x4-d1_x2) + (P4(2)-P2(2))*(d1_y4-d1_y2))/d;
          d5_d = ((P4(1) - P2(1))*(d5_x4-d5_x2) + (P4(2)-P2(2))*(d5_y4-d5_y2))/d;
          d1_b = d1_d - d1_d*(obj.l2^2 - obj.l3^2 + d^2)/(2*d^2);
          d5_b = d5_d - d5_d*(obj.l2^2 - obj.l3^2 + d^2)/(2*d^2);
          d1_h = -b*d1_b/h;
          d5_h = -b*d5_b/h;
          d1_yh = d1_y2 + (d1_b*d - d1_d*b)*(P4(2)-P2(2))/(d^2) + b*(d1_y4 - d1_y2)/d;
          d5_yh = d5_y2 + (d5_b*d - d5_d*b)*(P4(2)-P2(2))/(d^2) + b*(d5_y4 - d5_y2)/d;
          d1_xh = d1_x2 + (d1_b*d - d1_d*b)*(P4(1)-P2(1))/(d^2) + b*(d1_x4 - d1_x2)/d;
          d5_xh = d5_x2 + (d5_b*d - d5_d*b)*(P4(1)-P2(1))/(d^2) + b*(d5_x4 - d5_x2)/d;
          d1_y3 = d1_yh - h*(d1_x4 - d1_x2)/d - (d1_h*d - d1_d*h)*(P4(1)-P2(1))/(d^2);
          d1_x3 = d1_xh + h*(d1_y4 - d1_y2)/d + (d1_h*d - d1_d*h)*(P4(2)-P2(2))/(d^2);
          d5_y3 = d5_yh - h*(d5_x4 - d5_x2)/d - (d5_h*d - d5_d*h)*(P4(1)-P2(1))/(d^2);
          d5_x3 = d5_xh + h*(d5_y4 - d5_y2)/d + (d5_h*d - d5_d*h)*(P4(2)-P2(2))/(d^2);
          J = [d1_x3, d5_x3; d1_y3, d5_y3];
        end
        function m = manipulability(obj,J)
            [U, S, V] = svd(J);
%%
            m = max(diag(S)) + min(diag(S));
        end
        function vectors = v_manip_axes(obj, thetas)
            j = obj.jacobian(thetas);
            [vectors, values] = eig(j*j');
            vectors = sqrt(diag(values)).*vectors;
            vectors = [vectors, -vectors];
            display(vectors);
            
        end
      end
end


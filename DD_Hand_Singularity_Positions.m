function singularity_vals = DD_Hand_Singularity_Positions()
    linkage = FiveBarLinkage(50/1000,20/1000,30/1000,45/1000);
    
    A = [1; 1; 1; 1; 1]';
    b = 2*pi;
    lb = [0; 0; 0; 0; 0];
    ub = [pi; pi; pi; pi; pi];
    accum_theta = [];
    %options = optimoptions('fmincon','Display','off');
    accum_theta = [accum_theta; fmincon(@(theta)error_function(linkage, theta), [0  ,  0], [], [], [],[], lb, ub)];
    
    accum_theta = [accum_theta; fmincon(@(theta)error_function(linkage, theta), [1.4641,    1.3014], [], [], [],[], lb, ub)];

    accum_theta = [accum_theta;fmincon(@(theta)error_function(linkage, theta), [1.5147 ,   1.4301], [], [], [],[], lb, ub)]; 
    accum_theta = [accum_theta;fmincon(@(theta)error_function(linkage, theta), [1.5236    1.4525], [], [], [],[], lb, ub)];

    accum_theta = [accum_theta;fmincon(@(theta)error_function(linkage, theta), [1.5368    1.4857], [], [], [],[], lb, ub)];

    figure();
    hold on;   
    plot(accum_theta(:,1), accum_theta(:,2), "rx", "LineWidth",1);
    xlabel("x-coordinate (m)");
    ylabel("y-coordinate (m)");
    title("DD Hand Workspace");
    singularity_vals = [];
    for i = 1:(size(accum_theta(:,1)))
        singularity_vals = [singularity_vals,linkage.fk(accum_theta(i,:))];
    end
    
   
    
    function err = error_function(linkage, theta)
        A = linkage.l2*sin(theta(2)) - linkage.l1*sin(theta(1));
        B = linkage.l2*cos(theta(2)) - linkage.l1*cos(theta(1));
        C = (power(linkage.l3,2) - power(linkage.l1,2) - power(linkage.l2,2)...
        - power(linkage.l4,2) + 2*linkage.l1*linkage.l2*cos(theta(2)-theta(1)))...
              /(2*linkage.l4);
        D = sqrt(power(A,2) + power(B,2));

        err = asin(C/D).^2 - atan2(B,A).^2;
    end
end

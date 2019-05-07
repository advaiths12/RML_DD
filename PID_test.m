%include the dynamics of a 2 link robot, 
%add a mass and gravity and check the deflection
%use ODE45 for the dynamics of the 2 link arm 
P = .125;
I = .00005;
D = .7;
linkage = FiveBarLinkage(45/1000,30/1000,20/1000,50/1000);
set_point = [-.02, .1]; %meters in workspace
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
while 1
    pause(.01);
    time = time + 1;
    curr_theta = curr_theta + new_thetas_veloc;
    curr_point = linkage.fk(curr_theta);
    last_error = err;
    err = set_point - curr_point
    I_error = I_error + err;
    if(I_error < I_error_min) 
        I_error = I_error_min;
    end
    if(I_error > I_error_max) 
        I_error = I_error_max;
    end
    new_thetas_veloc = P*err + I*I_error - D*(err - last_error);
    plot(curr_point(1), curr_point(2), "g+");
    plot(set_point(1), set_point(2), "r.");
end
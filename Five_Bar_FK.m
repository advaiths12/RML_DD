function out1 = Five_Bar_FK(t,in2,in3,in4,g)
%FIVE_BAR_FK
%    OUT1 = FIVE_BAR_FK(T,IN2,IN3,IN4,G)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    08-May-2019 17:39:37

a1 = in4(1,:);
a2 = in4(2,:);
a3 = in4(3,:);
a4 = in4(4,:);
a5 = in4(5,:);
t1 = in2(1,:);
t5 = in2(3,:);
t3 = cos(t1);
t4 = a1.*t3;
t10 = sin(t1);
t11 = a1.*t10;
t12 = sin(t5);
t13 = a4.*t12;
t6 = t11-t13;
t8 = cos(t5);
t9 = a4.*t8;
t7 = a5-t4+t9;
t14 = t6.^2;
t15 = t7.^2;
t16 = t14+t15;
t17 = 1.0./t16;
t18 = a2.^2;
t19 = a3.^2;
t20 = t14+t15+t18-t19;
t21 = 1.0./sqrt(t16);
t22 = t20.^2;
t23 = t18-(t17.*t22)./4.0;
t24 = sqrt(t23);
out1 = reshape([0.0,0.0,t4,t11,t4+(t7.*t17.*t20)./2.0+t6.*t21.*t24,t11-(t6.*t17.*t20)./2.0+t7.*t21.*t24,a5+t9,t13,a5,0.0],[2,5]);

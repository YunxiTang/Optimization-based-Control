function B = autogen_B(q1,q2,q3,q4)
%AUTOGEN_B
%    B = AUTOGEN_B(Q1,Q2,Q3,Q4)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    06-Oct-2021 10:57:35

t2 = -q3;
t3 = -q4;
t4 = q1+t2;
t5 = q2+t3;
t6 = t4.^2;
t7 = t5.^2;
t8 = t6+t7;
t9 = 1.0./t8;
t10 = 1.0./sqrt(t8);
t11 = t4.*t9;
t12 = t5.*t9;
t13 = t4.*t10;
t14 = t5.*t10;
B = reshape([t13,t14,-t13,-t14,t12,-t11,-t12,t11],[4,2]);

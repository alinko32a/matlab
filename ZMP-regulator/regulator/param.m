clear;

sT = 0.01;
zc = 0.27;
g = 0.98;
omega = sqrt(g/zc);

A = [0 1; omega^2 0];
B = [0; -omega^2];
C = eye(2);
D = [0 0]';

%—¼‘«x
iniXzmin = -0.07;
iniXzmax = 0.07;
%•Ğ‘«x
%iniXzmin = 0.01;
%iniXzmax = 0.07;

%dS‚Ì‰Šú’l
inix = [0.00 0.0]';
%inix = [0.04 -0.4]';

q1 = -1;
q2 = -0.5;
K = -place(A, B, [q1 q2]);

%{
q1 = -0.2;
q2 = -0.6;
k1 = -q1*q2-1;
k2 = (q1+q2)/omega;
K = [k1 k2];
%}







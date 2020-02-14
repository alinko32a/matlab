clear;

sim_robot_param;

sT = 0.001;
%sT = 0.02;

%zc = 0.24;
zc = 0.1264;
%calcCoM();

g = 9.80665;
w = sqrt(g/zc);

A = [0 1; w^2 0];
B = [0; -w^2];
C = [w^2 0];
D = -w^2;

%倒立振り子(離散時間）
ss_G = ss(A,B,C,D);
ss_Gd = c2d(ss_G,sT,'zoh');
%ss_Gd = c2d(ss_G,sT,'tustin');
[Ad,Bd,Cd,Dd] = ssdata(ss_Gd);

%プロセスノイズの分散
Q = 1000;
%観測ノイズの分散
R = 1;

C = eye(2);
D = [0 0]';

%両足支持
iniYzmin = -0.0625;
iniYzmax = 0.0625;
%片足支持
%iniYzmin = 0.01;
%iniYzmax = 0.07;

iniXzmin = -0.05;
iniXzmax = 0.05;

%重心の初期値
iniy = [0.00 0.00]';
%iniy = [-0.01 0]';
%iniy = [0.00 -0.45]';

inix = [0.00 0.00]';

%{
q1 = -1;
q2 = -0.5;
K = -place(A, B, [q1 q2]);
%}

q1 = -1;
q2 = -0.5;
k1 = -q1*q2-1;
k2 = (q1+q2)/w;
K = [-k1 -k2];





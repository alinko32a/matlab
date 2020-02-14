
clear;
clc;

dt = 0.01;
zc = 0.27;
g = 0.98;
w = sqrt(g/zc);

q1 = - dt^3*w^3 + dt^2*w^2;
q2 = - dt^2*w^3 + dt*w^2;

a = - (dt^5*w^6 + dt^3*w^4 - q2*dt^2*w^2 - q1*dt*w^2 + q1*q2)/(dt^5*w^6) - (- 2*dt^2*w^2 + q2*dt + q1)/(dt^3*w^3)

format compact
%{ 
A = [ -3  1
       2 -2 ];
B = [ 2
      0 ];
 
p = [ -8+4j
      -8-4j ];
K = - place(A,B,p)
 
eig(A + B*K)
%}

sT = 0.01;
zc = 0.27;
g = 0.98;
omega = sqrt(g/zc);

A = [0 1; omega^2 0];
B = [0; -omega^2];
C = eye(2);
D = [0 0]';
ss_G = ss(A,B,C,D);
ss_Gd = c2d(ss_G,sT,'zoh');
%ss_Gd = c2d(ss_G,sT,'tustin');
[A,B,C,D] = ssdata(ss_Gd);

rankU = rank(ctrb(A, B))
eig(A)

q1 = -8+4j;
q2 = -8-4j;
K = -place(A, B, [q1 q2])

eig(A + B*K)

clear;
clc;
syms lambda w k1 k2 q1 q2 dt q
%% —£UŠÔ‚Ì“|—§U‚èq
%f = (lambda - 1)*(lambda + dt*w^2*k2 + 1) - (-dt)*(-dt*w^2*(1-k1)) == 0;
f = lambda^2 - 2*lambda - dt^2*w^2 - dt*k2*w^2 + dt^2*k1*w^2 + dt*k2*lambda*w^2 + 1 == 0;

%lambda = q1,q2‚Æ‚·‚é
f1 = subs(f, lambda, (1/(dt^2*w^2))*q1);
f2 = subs(f, lambda, (1/(dt*w^2))*q2);
[a_k1, a_k2] = solve(f1, f2, k1, k2)

%% Å“KƒŒƒMƒ…ƒŒ[ƒ^‚ÌğŒ•û’ö®
rg_f = w*k2 - k1 == 0;
%•û’ö®‚Ék1,k2‚ğ‘ã“ü
rg_f1 = subs(rg_f, [k1, k2], [a_k1, a_k2])
[solq1, solq2, param, cond] = solve(rg_f1, [q1 q2], 'ReturnConditions', true)
%solve(rg_f1)

%root(rg_f1, q1, q2)
%collect(rg_f1, [w dt])

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
%{
q1 = -0.00001;
q2 = -0.00005;
K = -place(A, B, [q1 q2]);
%}
Ad = [1 dt; dt*w^2 1];
Bd= [0; -dt*w^2];
Kd = [k1 k2];
Acl = Ad + Bd*Kd
%lambda = eig(Acl)
det(lambda*eye(2) - Acl)

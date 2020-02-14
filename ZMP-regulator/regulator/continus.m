clear;
clc;
syms lambda w k1 k2 q1 q2
%solve((lambda+2-2*k1)*(lambda+2) == 0, lambda)
%% 連続時間の倒立振り子
f = lambda*(lambda - w^2*k2) - w^2*(k1 + 1) == 0;

%lambda = wq1,wq2とする
f1 = subs(f, lambda, w*q1);
f2 = subs(f, lambda, w*q2);

%% 代入を繰り返してk1,k2を求める
%{
f_k2 = solve(f1, k2);
f3 = subs(f2, k2, f_k2);
a_k1 = solve(f3, k1);
a_k2 = solve(subs(f_k2 == k2, k1, a_k1), k2);
%}

%上記を代入なしで一回で求める
[a_k1, a_k2] = solve(f1, f2, k1, k2);

%% 最適レギュレータの条件方程式
rg_f = w*k2 - k1 == 0;
%方程式にk1,k2を代入
rg_f1 = subs(rg_f, [k1, k2], [a_k1, a_k2]);
%根
solve(rg_f1);
%simplify(rg_f1)
%式を因数分解
rg_f = w*k2 - k1;
factor(subs(rg_f, [k1, k2], [a_k1, a_k2]));

%% なんで、lambda = wq1,wq2にしたのか？
%lambda = q1,q2で試しに計算してみる
f1 = subs(f, lambda, q1);
f2 = subs(f, lambda, q2);
[a_k1, a_k2] = solve(f1, f2, k1, k2);
rg_f = w*k2 - k1 == 0;
%以下の代入で式にwが残る。lambda = wq1,wq2とすることで、式からwが消えるから、「lambda = wq1,wq2とする」
rg_f1 = subs(rg_f, [k1, k2], [a_k1, a_k2]);
collect(rg_f1, w);

A = [0 1; w^2 0];
B = [0; -w^2];
K = [k1 k2];

Acl = A + B*K
det(lambda*eye(2) - Acl)

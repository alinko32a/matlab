clear;
clc;
syms lambda w k1 k2 q1 q2
%solve((lambda+2-2*k1)*(lambda+2) == 0, lambda)
%% �A�����Ԃ̓|���U��q
f = lambda*(lambda - w^2*k2) - w^2*(k1 + 1) == 0;

%lambda = wq1,wq2�Ƃ���
f1 = subs(f, lambda, w*q1);
f2 = subs(f, lambda, w*q2);

%% ������J��Ԃ���k1,k2�����߂�
%{
f_k2 = solve(f1, k2);
f3 = subs(f2, k2, f_k2);
a_k1 = solve(f3, k1);
a_k2 = solve(subs(f_k2 == k2, k1, a_k1), k2);
%}

%��L�����Ȃ��ň��ŋ��߂�
[a_k1, a_k2] = solve(f1, f2, k1, k2);

%% �œK���M�����[�^�̏���������
rg_f = w*k2 - k1 == 0;
%��������k1,k2����
rg_f1 = subs(rg_f, [k1, k2], [a_k1, a_k2]);
%��
solve(rg_f1);
%simplify(rg_f1)
%������������
rg_f = w*k2 - k1;
factor(subs(rg_f, [k1, k2], [a_k1, a_k2]));

%% �Ȃ�ŁAlambda = wq1,wq2�ɂ����̂��H
%lambda = q1,q2�Ŏ����Ɍv�Z���Ă݂�
f1 = subs(f, lambda, q1);
f2 = subs(f, lambda, q2);
[a_k1, a_k2] = solve(f1, f2, k1, k2);
rg_f = w*k2 - k1 == 0;
%�ȉ��̑���Ŏ���w���c��Blambda = wq1,wq2�Ƃ��邱�ƂŁA������w�������邩��A�ulambda = wq1,wq2�Ƃ���v
rg_f1 = subs(rg_f, [k1, k2], [a_k1, a_k2]);
collect(rg_f1, w);

A = [0 1; w^2 0];
B = [0; -w^2];
K = [k1 k2];

Acl = A + B*K
det(lambda*eye(2) - Acl)

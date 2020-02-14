clear;

sT = 0.01;
zc = 0.27;
g = 0.98;
w = sqrt(g/zc);

%両足支持
iniXzmin = -0.07;
iniXzmax = 0.07;
%片足支持
%iniXzmin = 0.01;
%iniXzmax = 0.07;

%重心の初期値
inix = [0.0 0.0]';
%inix = [0.04 -0.4]';

%倒立振り子(連続時間）
A = [0 1; w^2 0];
B = [0; -w^2];
C = [w^2 0];
D = -w^2;
%倒立振り子(離散時間）
ss_G = ss(A,B,C,D);
ss_Gd = c2d(ss_G,sT,'zoh');
%ss_Gd = c2d(ss_G,sT,'tustin');
[Ad,Bd,Cd,Dd] = ssdata(ss_Gd);

%可制御
%rankU = rank(ctrb(A, B))
%Aの極は不安定
%eig(A)

%フィードバックで極を安定にする（設計は連続時間で行う）
q1 = -1;
q2 = -0.5;
K = -place(A, B, [q1 q2]);

%上記で指定した極になった
%eig(A + B*K)

%プロセスノイズの分散
Q = 0;
%観測ノイズの分散
R = 0.001;





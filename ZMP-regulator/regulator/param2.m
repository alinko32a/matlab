clear;

sT = 0.01;
zc = 0.27;
g = 0.98;
omega = sqrt(g/zc);


%A = [0 1; omega^2 0];
%B = [0; -omega^2];
A = [1 sT; sT*omega^2 1];
B = [0; -sT*omega^2];
%C = [omega^2 0];
%D = -omega^2;
C = eye(2);
D = [0 0]';


%{
A = [0 1; omega^2 0];
B = [0; -omega^2];
C = eye(2);
D = [0 0]';
ss_G = ss(A,B,C,D);
ss_Gd = c2d(ss_G,sT,'zoh');
%ss_Gd = c2d(ss_G,sT,'tustin');
[A,B,C,D] = ssdata(ss_Gd);
%}

%unco =0‚È‚ç‰Â§Œä
Co = ctrb(A,B);
unco = length(A) - rank(Co)
%unco =0‚È‚ç‰ÂŠÏ‘ª
Ob = obsv(A,C);
unob = length(A)-rank(Ob)

%—¼‘«x
iniXzmin = -0.07;
iniXzmax = 0.07;
%•Ğ‘«x
%iniXzmin = 0.01;
%iniXzmax = 0.07;

%dS‚Ì‰Šú’l
inix = [0.05 0.0]';
%inix = [0.04 -0.4]';


%q1 = -1;
%q2 = -0.5;
q1 = 3.5605e-04;
q2 = 0.0356;
k1 = -q1*q2-1;
k2 = (q1+q2)/omega;
K = [k1 k2];


q1 = -1;
q2 = -0.5;
K = -place(A, B, [q1 q2]);
%K = -placeAckermann2d(A, B, [q1 q2 q3]);
%K = -placeMIMO2D(A, B, [q1 q2]);

%{
Q = diag([0.1 0.1]);
H = 10;
K = dlqr(A,B,Q,H);
%}

%{
function F = placeAckermann2d(A,B,pole)
p = - pole;
d1 = p(1)+p(2)+p(3);
d0 = p(1)*p(2)*p(3);
X = A^2 + d1*A + d0*eye(3);

Uc = [B A*B];

F = [0 1]/Uc*X;
end

function F = placeMIMO2D(A,B,pole)
[u1 w1 v1] = svd([pole(1)*eye(2)-A B])
[u2 w2 v2] = svd([pole(2)*eye(2)-A B])
en = size(w1,2)
VV = [v1(:,3:en) v2(:,3:en)];
F = VV(3:en,:)/VV(1:2,:)
end
%}




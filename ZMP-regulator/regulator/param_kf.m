clear;

sT = 0.01;
zc = 0.27;
g = 0.98;
w = sqrt(g/zc);

%�����x��
iniXzmin = -0.07;
iniXzmax = 0.07;
%�Б��x��
%iniXzmin = 0.01;
%iniXzmax = 0.07;

%�d�S�̏����l
inix = [0.0 0.0]';
%inix = [0.04 -0.4]';

%�|���U��q(�A�����ԁj
A = [0 1; w^2 0];
B = [0; -w^2];
C = [w^2 0];
D = -w^2;
%�|���U��q(���U���ԁj
ss_G = ss(A,B,C,D);
ss_Gd = c2d(ss_G,sT,'zoh');
%ss_Gd = c2d(ss_G,sT,'tustin');
[Ad,Bd,Cd,Dd] = ssdata(ss_Gd);

%����
%rankU = rank(ctrb(A, B))
%A�̋ɂ͕s����
%eig(A)

%�t�B�[�h�o�b�N�ŋɂ�����ɂ���i�݌v�͘A�����Ԃōs���j
q1 = -1;
q2 = -0.5;
K = -place(A, B, [q1 q2]);

%��L�Ŏw�肵���ɂɂȂ���
%eig(A + B*K)

%�v���Z�X�m�C�Y�̕��U
Q = 0;
%�ϑ��m�C�Y�̕��U
R = 0.001;






clear;
sT = 0.03;

%% シミュレーション
%床面法線方向の仮想ばね定数[N/m]（ばねの硬さ 床の硬さ）
kn = 5000;
%床面法線方向の仮想ダンパの粘性係数[N*s/m]（クッション 小さくすると、乗ってる物が沈む）
bn = 100;
%床面接線方向の仮想ばね定数[N/m]
kt = 10;
%床面接線方向の仮想ダンパの粘性係数[N*s/m]
bt = 10;

%動摩擦係数
%mu_d = 100000000.4;
mu_d = 0.2;
%静止摩擦係数
%mu_s = 1000000000.6;
mu_s = 0.4;
%すべり速度のしきい値[m/s]
zvt = 0.01;

kn = 5000;
kt = 5000;
bn = 50;
bt = 50;
mu_d = 0.8;
mu_s = 2;
zvt = 1e-3;


%% General parameters
density = 1000;
foot_density = 2000;
world_damping = 0.25;
world_rot_damping = 0.25;

%% Contact/friction parameters
height_plane = 0.025;
plane_x = 3;
plane_y = 1;

%% Foot parameters
foot_x = 0.07;
foot_y = 0.045;
foot_z = 0.035;


%% Leg parameters
leg_radius = 0.022;
%lower_leg_length = 0.102;
%upper_leg_length = 0.102;
lower_leg_length = 0.095+0.01;
upper_leg_length = 0.095+0.01;

%% Torso parameters
torso_y = 0.12;
torso_x = 0.04;
torso_z = 0.12;
torso_offset_z = -torso_z/2;
torso_offset_x = 0.0;
%{
init_height = foot_z + lower_leg_length + upper_leg_length + ...
              torso_z/2 + torso_offset_z + height_plane/2;
%}          

init_height = foot_z + lower_leg_length + upper_leg_length + torso_z/2;

              
%% Joint parameters
joint_damping = 1;
joint_stiffness = 1;
%motion_time_constant = 0.01; %0.025;

%%
elems(1) = Simulink.BusElement;
elems(1).Name = 'name';
elems(1).DataType = 'Enum:JointName';
elems(1).Complexity = 'real';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).SampleTime = -1;

elems(2) = Simulink.BusElement;
elems(2).Name = 'm';
elems(2).DataType = 'double';
elems(2).Complexity = 'real';
elems(2).Dimensions = [1 1];
elems(2).DimensionsMode = 'Fixed';
elems(2).SampleTime = -1;

elems(3) = Simulink.BusElement;
elems(3).Name = 'sister';
elems(3).DataType = 'double';
elems(3).Complexity = 'real';
elems(3).Dimensions = [1 1];
elems(3).DimensionsMode = 'Fixed';
elems(3).SampleTime = -1;

elems(4) = Simulink.BusElement;
elems(4).Name = 'child';
elems(4).DataType = 'double';
elems(4).Complexity = 'real';
elems(4).Dimensions = [1 1];
elems(4).DimensionsMode = 'Fixed';
elems(4).SampleTime = -1;

elems(5) = Simulink.BusElement;
elems(5).Name = 'b';
elems(5).DataType = 'double';
elems(5).Complexity = 'real';
elems(5).Dimensions = [3 1];
elems(5).DimensionsMode = 'Fixed';
elems(5).SampleTime = -1;

elems(6) = Simulink.BusElement;
elems(6).Name = 'a';
elems(6).DataType = 'double';
elems(6).Complexity = 'real';
elems(6).Dimensions = [3 1];
elems(6).DimensionsMode = 'Fixed';
elems(6).SampleTime = -1;

elems(7) = Simulink.BusElement;
elems(7).Name = 'q';
elems(7).DataType = 'double';
elems(7).Complexity = 'real';
elems(7).Dimensions = [1 1];
elems(7).DimensionsMode = 'Fixed';
elems(7).SampleTime = -1;

elems(8) = Simulink.BusElement;
elems(8).Name = 'mother';
elems(8).DataType = 'double';
elems(8).Complexity = 'real';
elems(8).Dimensions = [1 1];
elems(8).DimensionsMode = 'Fixed';
elems(8).SampleTime = -1;

elems(9) = Simulink.BusElement;
elems(9).Name = 'dq';
elems(9).DataType = 'double';
elems(9).Complexity = 'real';
elems(9).Dimensions = [1 1];
elems(9).DimensionsMode = 'Fixed';
elems(9).SampleTime = -1;

elems(10) = Simulink.BusElement;
elems(10).Name = 'ddq';
elems(10).DataType = 'double';
elems(10).Complexity = 'real';
elems(10).Dimensions = [1 1];
elems(10).DimensionsMode = 'Fixed';
elems(10).SampleTime = -1;

elems(11) = Simulink.BusElement;
elems(11).Name = 'c';
elems(11).DataType = 'double';
elems(11).Complexity = 'real';
elems(11).Dimensions = [3 1];
elems(11).DimensionsMode = 'Fixed';
elems(11).SampleTime = -1;

elems(12) = Simulink.BusElement;
elems(12).Name = 'I';
elems(12).DataType = 'double';
elems(12).Complexity = 'real';
elems(12).Dimensions = [3 3];
elems(12).DimensionsMode = 'Fixed';
elems(12).SampleTime = -1;

elems(13) = Simulink.BusElement;
elems(13).Name = 'Ir';
elems(13).DataType = 'double';
elems(13).Complexity = 'real';
elems(13).Dimensions = [1 1];
elems(13).DimensionsMode = 'Fixed';
elems(13).SampleTime = -1;

elems(14) = Simulink.BusElement;
elems(14).Name = 'gr';
elems(14).DataType = 'double';
elems(14).Complexity = 'real';
elems(14).Dimensions = [1 1];
elems(14).DimensionsMode = 'Fixed';
elems(14).SampleTime = -1;

elems(15) = Simulink.BusElement;
elems(15).Name = 'u';
elems(15).DataType = 'double';
elems(15).Complexity = 'real';
elems(15).Dimensions = [1 1];
elems(15).DimensionsMode = 'Fixed';
elems(15).SampleTime = -1;

elems(16) = Simulink.BusElement;
elems(16).Name = 'R';
elems(16).DataType = 'double';
elems(16).Complexity = 'real';
elems(16).Dimensions = [3 3];
elems(16).DimensionsMode = 'Fixed';
elems(16).SampleTime = -1;

elems(17) = Simulink.BusElement;
elems(17).Name = 'p';
elems(17).DataType = 'double';
elems(17).Complexity = 'real';
elems(17).Dimensions = [3 1];
elems(17).DimensionsMode = 'Fixed';
elems(17).SampleTime = -1;

elems(18) = Simulink.BusElement;
elems(18).Name = 'vertex';
elems(18).DataType = 'double';
elems(18).Complexity = 'real';
elems(18).Dimensions = [3 8];
elems(18).DimensionsMode = 'Fixed';
elems(18).SampleTime = -1;

elems(19) = Simulink.BusElement;
elems(19).Name = 'face';
elems(19).DataType = 'double';
elems(19).Complexity = 'real';
elems(19).Dimensions = [4 6];
elems(19).DimensionsMode = 'Fixed';
elems(19).SampleTime = -1;

elems(20) = Simulink.BusElement;
elems(20).Name = 'v';
elems(20).DataType = 'double';
elems(20).Complexity = 'real';
elems(20).Dimensions = [3 1];
elems(20).DimensionsMode = 'Fixed';
elems(20).SampleTime = -1;

elems(21) = Simulink.BusElement;
elems(21).Name = 'w';
elems(21).DataType = 'double';
elems(21).Complexity = 'real';
elems(21).Dimensions = [3 1];
elems(21).DimensionsMode = 'Fixed';
elems(21).SampleTime = -1;



ULINK = Simulink.Bus;
ULINK.Elements = elems;

clear elems;




%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global uLINK       % グローバル変数として，外部関数からの参照を可能にする
uLINK = Simulink.Bus.createMATLABStruct('ULINK',[],[1 13]);

%%% SetupBipedRobot.m
%%% ２足歩行ロボット構造データ：図2.19，図2.20参照
%%% 各フィールドの定義は「表2.1　リンク情報」を参照のこと

ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

uLINK(1).name = JointName.BODY;
uLINK(1).m = 0.223;
uLINK(1).sister = 0;
uLINK(1).child = 2;
uLINK(1).b = [0 0 torso_z]';
uLINK(1).a = UZ;
uLINK(1).q = 0;

uLINK(2).name = JointName.RLEG_J0;
uLINK(2).m = 0.323/2;
uLINK(2).sister = 8;
uLINK(2).child = 3;
uLINK(2).b = [0 -torso_y/2 0]';
uLINK(2).a = UZ;
uLINK(2).q = 0;

uLINK(3).name = JointName.RLEG_J1;
uLINK(3).m = 0.005;
uLINK(3).sister = 0;
uLINK(3).child = 4;
uLINK(3).b = [0 0 0]';
uLINK(3).a = UX;
uLINK(3).q = 0;

uLINK(4).name = JointName.RLEG_J2;
uLINK(4).m = 0.173;
uLINK(4).sister = 0;
uLINK(4).child = 5;
uLINK(4).b = [0 0 0]';
uLINK(4).a = UY;
uLINK(4).q = 0;

uLINK(5).name = JointName.RLEG_J3;
uLINK(5).m = 0.005;
uLINK(5).sister = 0;
uLINK(5).child = 6;
uLINK(5).b = [0 0 -upper_leg_length]';
uLINK(5).a = UY;
uLINK(5).q = 0;

uLINK(6).name = JointName.RLEG_J4;
uLINK(6).m = 0.116;
uLINK(6).sister = 0;
uLINK(6).child = 7;
uLINK(6).b = [0 0 -lower_leg_length]';
uLINK(6).a = UY;
uLINK(6).q = 0;

uLINK(7).name = JointName.RLEG_J5;
uLINK(7).m = 0.107;
uLINK(7).sister = 0;
uLINK(7).child = 0;
uLINK(7).b = [0 0 0]';
uLINK(7).a = UX;
uLINK(7).q = 0;

uLINK(8).name = JointName.LLEG_J0;
uLINK(8).m = 0.323/2;
uLINK(8).sister = 0;
uLINK(8).child = 9;
uLINK(8).b = [0 torso_y/2 0]';
uLINK(8).a = UZ;
uLINK(8).q = 0;

uLINK(9).name = JointName.LLEG_J1;
uLINK(9).m = 0.005;
uLINK(9).sister = 0;
uLINK(9).child = 10;
uLINK(9).b = [0 0 0]';
uLINK(9).a = UX;
uLINK(9).q = 0;

uLINK(10).name = JointName.LLEG_J2;
uLINK(10).m = 0.173;
uLINK(10).sister = 0;
uLINK(10).child = 11;
uLINK(10).b = [0 0 0]';
uLINK(10).a = UY;
uLINK(10).q = 0;

uLINK(11).name = JointName.LLEG_J3;
uLINK(11).m = 0.005;
uLINK(11).sister = 0;
uLINK(11).child = 12;
uLINK(11).b = [0 0 -upper_leg_length]';
uLINK(11).a = UY;
uLINK(11).q = 0;

uLINK(12).name = JointName.LLEG_J4;
uLINK(12).m = 0.116;
uLINK(12).sister = 0;
uLINK(12).child = 13;
uLINK(12).b = [0 0 -lower_leg_length]';
uLINK(12).a = UY;
uLINK(12).q = 0;

uLINK(13).name = JointName.LLEG_J5;
uLINK(13).m = 0.107;
uLINK(13).sister = 0;
uLINK(13).child = 0;
uLINK(13).b = [0 0 0]';
uLINK(13).a = UX;
uLINK(13).q = 0;

FindMother(1);   %　妹，娘リンクの情報をもとに母リンクを設定する

%% フィールドの追加
for n=1:length(uLINK)
    uLINK(n).dq     = 0;            % 関節速度   [rad/s]
    uLINK(n).ddq    = 0;            % 関節加速度 [rad/s^2]
    uLINK(n).c      = [0 0 0]';     % 重心位置   [m]
    uLINK(n).I      = zeros(3,3);   % 重心回りの慣性テンソル [kg.m^2]
    uLINK(n).Ir     = 0.0;          % モータの電気子慣性モーメント [kg.m^2]
    uLINK(n).gr     = 0.0;          % モータの減速比 [-]
    uLINK(n).u      = 0.0;          %  関節トルク [Nm]
end

%% リンクnameと同名の変数にID番号をセットする
for n=1:length(uLINK)
    %eval([uLINK(n).name,'=',num2str(n),';']);
    %{
    uLINK(n).R = RPY2R([0 0 0]);
    uLINK(n).p = [0 0 0]';
    %}
    uLINK(n).R = eye(3);
    uLINK(n).p = 0;
    uLINK(n).v = [0 0 0]';
    uLINK(n).w = [0 0 0]';
end

%% 胴体，および足部を剛体でモデル化
shape = [torso_x torso_y torso_z];     % 奥行き，幅，高さ [m]
com   = [0 0 0]';        % 重心位置
SetupRigidBody(JointName.BODY, shape,com);

shape = [foot_x foot_y foot_z];    % 奥行き，幅，高さ [m]
com   = [0 0 0]';   % 重心位置
SetupRigidBody(JointName.RLEG_J5, shape,com);

shape = [foot_x foot_y foot_z];     % 奥行き，幅，高さ [m]
com   = [0 0 0]';    % 重心位置
SetupRigidBody(JointName.LLEG_J5, shape,com);

%% 非特異姿勢へ移行

uLINK(JointName.RLEG_J2).q = -5.0*ToRad;
uLINK(JointName.RLEG_J3).q = 10.0*ToRad;
uLINK(JointName.RLEG_J4).q = -5.0*ToRad;

uLINK(JointName.LLEG_J2).q = -5.0*ToRad;
uLINK(JointName.LLEG_J3).q = 10.0*ToRad;
uLINK(JointName.LLEG_J4).q = -5.0*ToRad;

uLINK(JointName.BODY).p = [0.0, 0.0, 0.19]';
uLINK(JointName.BODY).R = RPY2R([0 0*ToRad 0]);

%ForwardKinematics(1);

Rfoot.p = [0 -torso_y/2 0]';
Rfoot.R = RPY2R([0 0 0]);
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';

Lfoot.p = [0 torso_y/2 0]';
Lfoot.R = RPY2R([0 0 0]);
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';

InverseKinematicsAll(7, Rfoot);
InverseKinematicsAll(13, Lfoot);
ForwardVelocity(1);


%%
%一歩に要する時間
Tsup = 0.8;
%歩行時の足最大高さ
szh = 0.02;
%歩幅
xw = 0.00;

%% 歩行パラメータ
right_yw = (-torso_y/2)+leg_radius;
left_yw = (torso_y/2)-leg_radius;

%{
Rx = [0 0 0 0 0 0 0 0 0 0];
Ry = [0 0 0 0 0 0 0 0 0 0];
%}

%{
Rx = [0 0 0 xw*1 xw*2 xw*3 xw*4 xw*5 xw*5 xw*5];
Ry = [0 0 left_yw right_yw left_yw right_yw left_yw right_yw 0 0];
%}

%Rx = [0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00 0.00];
Rx = [0.00 0.00 0.00 xw*1 xw*2 xw*3 xw*4 xw*5 xw*6 xw*7 xw*8 xw*9 xw*10 xw*10 xw*10 xw*10];
Ry = [0.00 0.00 left_yw right_yw left_yw right_yw left_yw right_yw left_yw right_yw left_yw right_yw left_yw 0.00 0.00 0.00];


RLen = length(Rx);

global M
M = TotalMass(1);


com = calcCoM;     % 重心位置
Zc  = com(3);      % 線形倒立振子の設定高さ


%init_height = 0.2;



%% 予見制御


%%
N = 1500;
%Zc = uLINK(JointName.BODY).p(3); 
g = 9.8;

%%
A = [0  1  0
     0  0  1
     0  0  0];
B = [ 0
      0
      1];
C = [ 1  0  -Zc/g];
D = 0;
ss_G = ss(A,B,C,D);

ss_Gd = c2d(ss_G,sT,'zoh');
[A,B,C,D] = ssdata(ss_Gd);

%%
PHI = [1  -C*A
    zeros(3,1) A];

G = [-C*B
    B];

GR = [1
    zeros(3,1)];

%%
Q = diag([1e5 1e2 1e2 1e2]);
H = 1;

[F0,P,e] = dlqr(PHI,G,Q,H);
F0 = F0*-1;

Fe = F0(:,1);
Fx = F0(:,2:4);

%%
XI = PHI+G*F0;

GammaRR = eye(N);
GammaRR = GammaRR.*(H+G'*P*G);

DeltaRR = zeros(N,1);
for i = 1:N
    DeltaRR(i,1) = G'*((XI')^(i-1))*P*GR;
end

FR = -(inv(GammaRR)*DeltaRR);

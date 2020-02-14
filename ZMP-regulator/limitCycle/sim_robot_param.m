


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
plane_x = 6;
plane_y = 1;

%% Foot parameters
foot_x = 0.07;
foot_y = 0.045;
foot_z = 0.01;


%% Leg parameters
leg_radius = 0.02;
lower_leg_length = 0.102;
upper_leg_length = 0.102;

%% Torso parameters
torso_y = 0.08;
torso_x = 0.1;
torso_z = 0.10;
torso_offset_z = -torso_z/2;
torso_offset_x = 0.0;
%{
init_height = foot_z + lower_leg_length + upper_leg_length + ...
              torso_z/2 + torso_offset_z + height_plane/2;
%}          

init_height = foot_z + lower_leg_length + upper_leg_length + torso_z/2;
%init_height = foot_z + lower_leg_length*sin(deg2rad(30)) + upper_leg_length*sin(deg2rad(30)) + torso_z/2;
              
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
uLINK = Simulink.Bus.createMATLABStruct('ULINK',[],[1 15]);

%%% SetupBipedRobot.m
%%% ２足歩行ロボット構造データ：図2.19，図2.20参照
%%% 各フィールドの定義は「表2.1　リンク情報」を参照のこと

UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

l1 = 0.03; l3 = 0.095; l4 = 0.095; l6 = 0.03;
th1 = deg2rad(0); th2 = deg2rad(0); th3 = deg2rad(30); th4 = deg2rad(-60); th5 = deg2rad(30); th6 = deg2rad(0);

%lower_leg_length = l4;
%upper_leg_length = l3;


%torso_z = 0.26-0.0121;
%torso_z = 0.30;

%%
uLINK(1).name = JointName.BODY;
uLINK(1).m = 0.0;
uLINK(1).sister = 0;
uLINK(1).child = 2;
uLINK(1).b = [0 0 0]';
%uLINK(1).a = [0 0 0]';
uLINK(1).a = UZ;
uLINK(1).q = 0;
uLINK(1).p = [0 0 torso_z]';
%%
uLINK(2).name = JointName.RLEG_J0;
uLINK(2).m = 0.06;
uLINK(2).sister = 9;
uLINK(2).child = 3;
uLINK(2).b = [0 -torso_y/2 0]';
uLINK(2).a = UZ;
uLINK(2).q = 0;
uLINK(2).p = [0 -torso_y/2 torso_z]';

uLINK(3).name = JointName.RLEG_J1;
uLINK(3).m = 0.06;
uLINK(3).sister = 0;
uLINK(3).child = 4;
uLINK(3).b = [0 0 -l1]';
uLINK(3).a = UX;
uLINK(3).q = 0;
uLINK(3).p = [0 0 torso_z-l1]';

uLINK(4).name = JointName.RLEG_J2;
uLINK(4).m = 0.1;
uLINK(4).sister = 0;
uLINK(4).child = 5;
uLINK(4).b = [0 0 0]';
uLINK(4).a = UY;
uLINK(4).q = 0;
uLINK(4).p = [0 0 torso_z-l1]';

uLINK(5).name = JointName.RLEG_J3;
uLINK(5).m = 0.1;
uLINK(5).sister = 0;
uLINK(5).child = 6;
uLINK(5).b = [0 0 -l3]';
uLINK(5).a = UY;
uLINK(5).q = 0;
uLINK(5).p = [0 0 torso_z-l1-l3]';

uLINK(6).name = JointName.RLEG_J4;
uLINK(6).m = 0.06;
uLINK(6).sister = 0;
uLINK(6).child = 7;
uLINK(6).b = [0 0 -l4]';
uLINK(6).a = UY;
uLINK(6).q = 0;
uLINK(6).p = [0 0 torso_z-l1-l3-l4]';

uLINK(7).name = JointName.RLEG_J5;
uLINK(7).m = 0.06;
uLINK(7).sister = 0;
uLINK(7).child = 8;
uLINK(7).b = [0 0 0]';
uLINK(7).a = UX;
uLINK(7).q = 0;
uLINK(7).p = [0 0 torso_z-l1-l3-l4]';

uLINK(8).name = JointName.RLEG_FOOT;
uLINK(8).m = 0.03;
uLINK(8).sister = 0;
uLINK(8).child = 0;
uLINK(8).b = [0 0 -l6]';
%uLINK(8).a = [0 0 0]';
uLINK(8).a = UZ;
uLINK(8).q = 0;
uLINK(8).p = [0 0 0]';

%%
uLINK(9).name = JointName.LLEG_J0;
uLINK(9).m = 0.06;
uLINK(9).sister = 0;
uLINK(9).child = 10;
uLINK(9).b = [0 torso_y/2 0]';
uLINK(9).a = UZ;
uLINK(9).q = 0;
uLINK(9).p = [0 torso_y/2 torso_z]';

uLINK(10).name = JointName.LLEG_J1;
uLINK(10).m = 0.06;
uLINK(10).sister = 0;
uLINK(10).child = 11;
uLINK(10).b = [0 0 -l1]';
uLINK(10).a = UX;
uLINK(10).q = 0;
uLINK(10).p = [0 0 torso_z-l1]';

uLINK(11).name = JointName.LLEG_J2;
uLINK(11).m = 0.1;
uLINK(11).sister = 0;
uLINK(11).child = 12;
uLINK(11).b = [0 0 0]';
uLINK(11).a = UY;
uLINK(11).q = 0;
uLINK(11).p = [0 0 torso_z-l1]';

uLINK(12).name = JointName.LLEG_J3;
uLINK(12).m = 0.1;
uLINK(12).sister = 0;
uLINK(12).child = 13;
uLINK(12).b = [0 0 -l3]';
uLINK(12).a = UY;
uLINK(12).q = 0;
uLINK(12).p = [0 0 torso_z-l1-l3]';

uLINK(13).name = JointName.LLEG_J4;
uLINK(13).m = 0.06;
uLINK(13).sister = 0;
uLINK(13).child = 14;
uLINK(13).b = [0 0 -l4]';
uLINK(13).a = UY;
uLINK(13).q = 0;
uLINK(13).p = [0 0 torso_z-l1-l3-l4]';

uLINK(14).name = JointName.LLEG_J5;
uLINK(14).m = 0.06;
uLINK(14).sister = 0;
uLINK(14).child = 15;
uLINK(14).b = [0 0 0]';
uLINK(14).a = UX;
uLINK(14).q = 0;
uLINK(14).p = [0 0 torso_z-l1-l3-l4]';

uLINK(15).name = JointName.LLEG_FOOT;
uLINK(15).m = 0.03;
uLINK(15).sister = 0;
uLINK(15).child = 0;
uLINK(15).b = [0 0 -l6]';
%uLINK(15).a = [0 0 0]';
uLINK(15).a = UZ;
uLINK(15).q = 0;
uLINK(15).p = [0 0 0]';

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
    uLINK(n).R = eye(3);
    uLINK(n).v = [0 0 0]';
    uLINK(n).w = [0 0 0]';
end

%ForwardKinematics(1);
%uLINK(8).p

%CalcJacobian 7行目、9行目 jsize→6に変更
%MoveJoints 5行目 length(idx)→6に変更
%InverseKinematicsAll 24行目 length(idx)→6に変更

%% 非特異姿勢へ移行

uLINK(JointName.RLEG_J2).q = deg2rad(30);
uLINK(JointName.RLEG_J3).q = deg2rad(-60);
uLINK(JointName.RLEG_J4).q = deg2rad(30);

uLINK(JointName.LLEG_J2).q = deg2rad(30);
uLINK(JointName.LLEG_J3).q = deg2rad(-60);
uLINK(JointName.LLEG_J4).q = deg2rad(30);

uLINK(JointName.BODY).p = [0.00, 0.00, l1+cos(deg2rad(30))*l3+cos(deg2rad(30))*l4+l6]';
uLINK(JointName.BODY).R = RPY2R([0 0 0]);

%ForwardKinematics(1);

Rfoot.p = [0 -torso_y/2 0]';
Rfoot.R = RPY2R([0 0 0]);
Rfoot.v = [0 0 0]';
Rfoot.w = [0 0 0]';
InverseKinematicsAll(8, Rfoot);

Lfoot.p = [0 torso_y/2 0]';
Lfoot.R = RPY2R([0 0 0]);
Lfoot.v = [0 0 0]';
Lfoot.w = [0 0 0]';
InverseKinematicsAll(15, Lfoot);





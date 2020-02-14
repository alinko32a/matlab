clear

p = -0.05;
l1 = -0.03; l3 = -0.1; l4 = -0.1; l6 = -0.03;
%l1 = 0.03; l3 = 0.1; l4 = 0.1; l6 = 0.03;
th1 = deg2rad(0); th2 = deg2rad(0); th3 = deg2rad(20+30); th4 = deg2rad(-40); th5 = deg2rad(20); th6 = deg2rad(30);

%ワールド座標から見た関節1(腰)の位置
pw1 = [0 0 0.26-0.0121]';
%pw1 = [0 0 0]';
Tw1 = [diag([1 1 1]),pw1;
    zeros(1,3),1];

%関節1の回転
T1 = yaw_transformation(th1,[0,0,0]');

%関節1から見た関節2の位置と回転
T12 = roll_transformation(th2,[0,0,l1]');
%ワールド座標から見た関節2上での質点の位置
p2 = [0 0 p 1]';
pw2 = Tw1*T1*T12*p2;

%関節2から見た関節3の位置と回転
T23 = pitch_transformation(th3,[0,0,0]');
%ワールド座標から見た関節3上での質点の位置
p3 = [0 0 p 1]';
pw3 = Tw1*T1*T12*T23*p3;

%関節3から見た関節4の位置と回転
T34 = pitch_transformation(th4,[0,0,l3]');
%ワールド座標から見た関節4上での質点の位置
p4 = [0 0 p 1]';
pw4 = Tw1*T1*T12*T23*T34*p4;

%関節4から見た関節5の位置と回転
T45 = pitch_transformation(th5,[0,0,l4]');
%ワールド座標から見た関節5上での質点の位置
p5 = [0 0 p 1]';
pw5 = Tw1*T1*T12*T23*T34*T45*p5;

%関節5から見た関節6の位置と回転
T56 = roll_transformation(th6,[0,0,0]');
%ワールド座標から見た関節6上での質点(足裏)の位置
pt = [0 0 l6 1]';
pwt = Tw1*T1*T12*T23*T34*T45*T56*pt

function T = roll_transformation(th,p)

    R = [1,0,0;
        0,cos(th),-sin(th);
        0,sin(th),cos(th)];
    
    T = [R,p;
        zeros(1,3),1];
end

function T = pitch_transformation(th,p)

    R = [cos(th),0,sin(th);
        0,1,0;
        -sin(th),0,cos(th)];
    
    T = [R,p;
        zeros(1,3),1];
end

function T = yaw_transformation(th,p)

    R = [cos(th),-sin(th),0;
        sin(th),cos(th),0;
        0,0,1];
    
    T = [R,p;
        zeros(1,3),1];
end
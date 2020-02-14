clear

p = -0.05;
l1 = -0.03; l3 = -0.1; l4 = -0.1; l6 = -0.03;
%l1 = 0.03; l3 = 0.1; l4 = 0.1; l6 = 0.03;
th1 = deg2rad(0); th2 = deg2rad(0); th3 = deg2rad(20+30); th4 = deg2rad(-40); th5 = deg2rad(20); th6 = deg2rad(30);

%���[���h���W���猩���֐�1(��)�̈ʒu
pw1 = [0 0 0.26-0.0121]';
%pw1 = [0 0 0]';
Tw1 = [diag([1 1 1]),pw1;
    zeros(1,3),1];

%�֐�1�̉�]
T1 = yaw_transformation(th1,[0,0,0]');

%�֐�1���猩���֐�2�̈ʒu�Ɖ�]
T12 = roll_transformation(th2,[0,0,l1]');
%���[���h���W���猩���֐�2��ł̎��_�̈ʒu
p2 = [0 0 p 1]';
pw2 = Tw1*T1*T12*p2;

%�֐�2���猩���֐�3�̈ʒu�Ɖ�]
T23 = pitch_transformation(th3,[0,0,0]');
%���[���h���W���猩���֐�3��ł̎��_�̈ʒu
p3 = [0 0 p 1]';
pw3 = Tw1*T1*T12*T23*p3;

%�֐�3���猩���֐�4�̈ʒu�Ɖ�]
T34 = pitch_transformation(th4,[0,0,l3]');
%���[���h���W���猩���֐�4��ł̎��_�̈ʒu
p4 = [0 0 p 1]';
pw4 = Tw1*T1*T12*T23*T34*p4;

%�֐�4���猩���֐�5�̈ʒu�Ɖ�]
T45 = pitch_transformation(th5,[0,0,l4]');
%���[���h���W���猩���֐�5��ł̎��_�̈ʒu
p5 = [0 0 p 1]';
pw5 = Tw1*T1*T12*T23*T34*T45*p5;

%�֐�5���猩���֐�6�̈ʒu�Ɖ�]
T56 = roll_transformation(th6,[0,0,0]');
%���[���h���W���猩���֐�6��ł̎��_(����)�̈ʒu
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
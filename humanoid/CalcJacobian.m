function J = CalcJacobian(idx)
% Jacobian matrix of current configration in World frame
global uLINK

jsize = length(idx);
target = uLINK(idx(end)).p;   % absolute target position
J = zeros(6,6);

for n=1:6
    j = idx(n);
    a = uLINK(j).R * uLINK(j).a;  % joint axis vector in world frame
    J(:,n) = [cross(a, target - uLINK(j).p) ; a ];
end


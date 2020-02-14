function [px,py] = calcZMP(c,dP,dL,pz)
global M

g = 9.8;

px = (M*g*c(1) + pz * dP(1) - dL(2))/(M*g + dP(3));
py = (M*g*c(2) + pz * dP(2) + dL(1))/(M*g + dP(3));
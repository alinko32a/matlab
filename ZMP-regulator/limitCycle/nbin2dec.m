function [x]=nbin2dec(binstring)
% �����t������2�i������10�i���ɕϊ� (���̐���2�̕␔�ŕ\��)
% binstring�F2�i���ŕ\�����ꂽ������
% x        �F10�i��

n = length(binstring);
x = bin2dec(binstring);

if x >= 2^(n-1) 
    x=x-2^n;
end
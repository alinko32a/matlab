function mc = calcMC(j)
global uLINK

%ÉTÉCÉYå≈íË
coder.varsize('mc',[3 1]);

if j == 0
    mc = [0 0 0]';
else
    mc = uLINK(j).m * (uLINK(j).p + uLINK(j).R * uLINK(j).c);
    mc = mc + calcMC(uLINK(j).sister) + calcMC(uLINK(j).child);
end

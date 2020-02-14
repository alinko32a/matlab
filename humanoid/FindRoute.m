function idx = FindRoute(to)
% return the list of joint number connecting ROOT to 'to'
global uLINK

%ÉTÉCÉYå≈íË
coder.varsize('idx',[1 7]);

i = uLINK(to).mother;
if (i == 1)
    idx = [to];
else
    idx = [FindRoute(i) to];
end
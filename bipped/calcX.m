%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���̕���
%wt:
%t:
%T:
function x = calcX(wt,t,T) 

    x = 0;

    %���1
    if (0 <= t && t <= T/2) 
        x = 4*(wt*100)/T^2*t^2;
    end
    %���2
    if (T/2 < t && t <= T) 
       x = (wt*100)+4*(wt*100)/T*(t-T/2)-4*(wt*100)/T^2*(t-T/2)^2;
    end

    x = x/100;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%¡‚Ì‘«æ‚‚³
%wt:
%x:
%hT:
function z = calcZ(wt,x,hT)
    
    z = 0;
    
    if (wt ~= 0)
        if (0 <= x && x <= wt)
            z = (hT/wt)*x;
        end
        if (wt < x && x <= 2*wt) 
            z = hT-(hT/wt)*(x-wt);
        end
    else
        z = 0;
    end
end


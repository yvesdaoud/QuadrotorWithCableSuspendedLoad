function [xQ,vQ] = taut_2_nontaut(xL,q,vL,w,l)
    
    xQ = xL - l * q;
    vQ = vL - l * cross(w,q);
    
end


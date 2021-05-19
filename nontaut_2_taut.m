function [q,d_q,w] = nontaut_2_taut(xL,vL,xQ,vQ,l,q_0,dt)

    q = (xL - xQ) / l;
    
    d_q = (q - q_0) / dt;
    
    w = cross((vQ - vL),q) / l;

end


function [q_d, Fn] = load_position_controller(xL, xd, vL, vd, d2_xd, integral_xL, mQ, mL, g, l, q, d_q, kx, kv, kI)

    e_x = xL - xd;
    e_v = vL - vd;
    e_I = integral_xL;
    
    A = - kx*e_x - kv*e_v - kI*e_I + (mQ+mL)*(d2_xd + g*[0;0;1]) + mQ*l*dot(d_q,d_q)*q;
    
    Fn = dot(A,q) * q;
    
    q_d = - A / norm(A);

end


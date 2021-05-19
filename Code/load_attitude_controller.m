function [F, f] = load_attitude_controller(mQ, l, R, Fn, q, d_q, q_d, q_d_m1, q_d_m2, dt, kq, kw)
    
%     dq_d = (q_d - q_d_m1) / dt;
    dq_d = [0;0;0]; %simplified case
%     d2q_d = (q_d - 2*q_d_m1 + q_d_m2) / dt^2;
    d2q_d = [0;0;0]; %simplified case

    e_q = vec_2_skew(q)^2 * q_d;
    e_dq = d_q - cross( cross(q_d,dq_d), q );
    
    Fpd = - kq*e_q - kw*e_dq;
    
    Fff = mQ*l*dot( q, cross(q_d,dq_d) )*cross(q,d_q) + mQ*l*cross( cross(q_d,d2q_d), q );
    
    F = Fn - Fpd - Fff;

    %thrust unnormalized Fn
    f = dot( F, R*[0;0;1] );
    
    %for the moment normalized Fn
    Fn = - dot(q_d,q) * q;
    
    F = Fn - Fpd - Fff;
    
end


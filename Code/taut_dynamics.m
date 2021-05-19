function [xL, q, d_q, R, vL, w, Omega] = taut_dynamics(g, mL, mQ, l, JQ, Omega_0, R_0, M, q_0, d_q_0, f,...
                                                                            w_0, xL_0, vL_0, t, dt)
    
    %quadrotor attitude dynamics
    %angular velocity 
    [~, Omega_var] = ode45(@(t,Omega) odefunOmega(t,Omega,M,JQ), [t,t+dt], Omega_0);
    Omega = Omega_var(max(size(Omega_var)),:)';
    
    %rotation matrix
    skew_Omega = vec_2_skew(Omega);
    B = skew_Omega * dt;
    sigma = norm( Omega * dt );
    %https://www.cl.cam.ac.uk/techreports/UCAM-CL-TR-696.pdf
    R = R_0 * ( eye(3) + (sin(sigma)/sigma) * B + ((1-cos(sigma))/sigma^2) * B^2 );
    
    %load attitude dynamics
    %q uniti vector from quad to load
    [~, q_var] = ode45(@(t,q) odefunq(t,q,mQ,l,f,R), [t,t+dt],...
        [q_0(1),d_q_0(1),q_0(2),d_q_0(2),q_0(3),d_q_0(3)]);
    last_q = q_var(max(size(q_var)),:);
    q = [last_q(1),last_q(3),last_q(5)]';
    d_q = [last_q(2),last_q(4),last_q(6)]';
    
    %angular velocity of the load
    d_w = - cross(q,f*R*[0;0;1]) / (mQ*l);
    w = d_w * dt + w_0;
    
    %quadrotot position dynamics
    d_vL = ( ( dot(q,f*R*[0;0;1]) - mQ*l*dot(d_q,d_q) )* q / (mQ+mL) ) - g*[0;0;1];
    vL = d_vL * dt + vL_0;
    xL = 0.5 * d_vL * dt^2 + vL_0 * dt + xL_0;
    
end

function d_Omega = odefunOmega(t,Omega,M,JQ)

    temp1 = [Omega(1); Omega(2); Omega(3)];
    temp2 = JQ \ ( M - cross(temp1, JQ*temp1) );
    d_Omega(1) = temp2(1);
    d_Omega(2) = temp2(2);
    d_Omega(3) = temp2(3);
    d_Omega = d_Omega';

end

function d_q = odefunq(t,q,mQ,l,f,R)

    temp1 = [q(2); q(4); q(6)];
    d_q(1) = temp1(1);
    d_q(3) = temp1(2);
    d_q(5) = temp1(3);
    
    q1s = [q(1); q(3); q(5)];
    temp2 = (1/(mQ*l)) * (  cross(q1s,cross(q1s,f*R*[0;0;1])) - mQ*l*dot(temp1,temp1)*q1s );
    d_q(2) = temp2(1);
    d_q(4) = temp2(2);
    d_q(6) = temp2(3);
    d_q = d_q';
    
end

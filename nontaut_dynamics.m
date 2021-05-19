function [xL, xQ, R, vL, vQ, Omega] = nontaut_dynamics(mL, mQ, g, JQ, vL_0, xL_0, vQ_0, xQ_0, f,...
                                                            Omega_0, R_0, M, t, dt)
    
    %load dynamics
    d_vL = - (g*[0; 0; 1]) / mL;
    vL = d_vL * dt + vL_0;
    xL = 0.5 * d_vL * dt^2 + vL_0 * dt + xL_0;
    
    %quadrotor attitude dynamics
    %angular velocity 
    [~, Omega_var] = ode45(@(t,Omega) odefunOmega(t,Omega,M,JQ), [t,t+dt], Omega_0);
    Omega = Omega_var(max(size(Omega_var)),:)';
    
    %rotation matrix
    skew_Omega = vec_2_skew(Omega);
    B = skew_Omega * dt;
    sigma = norm( Omega * dt );
    R = R_0 * ( eye(3) + (sin(sigma)/sigma) * B + ((1-cos(sigma))/sigma^2) * B^2 );
    
    %quadrotor position dynamics
    d_vQ = f * R * [0; 0; 1] / mQ - g*[0; 0; 1];
    vQ =  d_vQ * dt + vQ_0;
    xQ = 0.5 * d_vQ * dt^2 + vQ_0 * dt + xQ_0;
    
end


function d_Omega = odefunOmega(t,Omega,M,JQ)

    temp1 = [Omega(1); Omega(2); Omega(3)];
    temp2 = JQ \ ( M - cross(temp1, JQ*temp1) );
    d_Omega(1) = temp2(1);
    d_Omega(2) = temp2(2);
    d_Omega(3) = temp2(3);
    d_Omega = d_Omega';

end

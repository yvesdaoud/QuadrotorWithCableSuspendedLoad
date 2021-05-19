function [M,R_d,Omega_d] = quadrotor_attitude_controller(JQ, F, b1, R, R_d_m1, dt, Omega, Omega_d_m1, kR, kOmega)
    
    b3_d = F / norm(F);
    
    b1_d = cross( b1, b3_d ) / norm(cross( b1, b3_d ));
    
    b2_d = cross( b3_d, b1_d ) / norm(cross( b3_d, b1_d ));
    
    R_d = [ b1_d, b2_d, b3_d ];
    
    
    %rates R_d
%     dR_d = (R_d - R_d_m1) / dt;
    dR_d = zeros(3,3); %simplified case
    
    skew_Omega_d = R_d' * dR_d;
    Omega_d = skew_2_vec(skew_Omega_d);
    
    %rates Omega_d
%     dOmega_d = (Omega_d - Omega_d_m1) / dt;
    dOmega_d = [0;0;0]; %simplified case
    
    %errors
    e_R = skew_2_vec( 0.5 * (R_d'*R - R'*R_d) );
    e_Omega = Omega - R'*R_d*Omega_d;
    
    M = - kR*e_R - kOmega*e_Omega + cross(Omega, JQ*Omega)...
                                    - JQ*( vec_2_skew(Omega)*R'*R_d*Omega_d - R'*R_d*dOmega_d );
    
end


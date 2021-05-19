function [phi, theta, psi] = rot_2_eul(R)
    
    % based on Diebel eq(361)
    
    R = R';
    
    % check if R is an orthogonal Rotation Matrix
    det_R = R(1,1)*(R(2,2)*R(3,3)-R(2,3)*R(3,2))-R(1,2)*(R(2,1)*R(3,3)...
                -R(2,3)*R(3,1))+R(1,3)*(R(2,1)*R(3,2)-R(2,2)*R(3,1));
    if ( abs(det_R-1) > 1e-4)
        warning("The matrix provided is not a Rotation Matrix, det(R)= " + abs(det_R-1));
    end
    
    %ZYX
    phi = asin(R(2,3));
    theta = atan2(-R(1,3),R(3,3));
    psi = atan2(-R(2,1),R(2,2));
    
end


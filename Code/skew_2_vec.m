%% extract a 3x1 vector from a skew symmetric matrix

function [w_vec] = skew_2_vec(w_skew)

    w_vec = [ w_skew(3,2) ; w_skew(1,3) ; w_skew(2,1) ];

end

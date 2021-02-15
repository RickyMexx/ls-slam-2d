## usage: [LM_P] = lateration (POSES_XY, P, R, L_NOISE)
##
## Computes the position of a landmark given a set
## of poses. A set of indices p(i) refers to the poses
## observing the landmark, sensing it at distance r(i).
## This form of triangulation derives the position of the
## landmark building circles of radius r around the poses,
## finding, whether is possible, their intersection.
## Further info: https://www.sciencedirect.com/science/article/pii/S1389128603003566
##
##  poses_xy : Poses of the robot in xy coordinates
##  p : Indices of the poses referring to current landmark
##  r : Set of range measurements from p-th pose
##  l_noise : Lateration noise added in the system calculation


function [lm_p] = lateration(poses_xy, p, r, l_noise)
    r_num = length(r);

    [xn, yn, rn] = deal(poses_xy(1,p(r_num)), poses_xy(2,p(r_num)), r(r_num));

    A = zeros(r_num-1, 2);
    b = zeros(r_num-1, 1);

    for k=1:r_num-1
        [xk, yk, rk] = deal(poses_xy(1,p(k)), poses_xy(2,p(k)), r(k));
        
        A(k,:) = 2 * [(xk - xn) (yk - yn)];
        b(k,:) = [xk^2-xn^2 + yk^2-yn^2 + rn^2-rk^2];
    endfor

    lm_p = (A'*A + eye(2)*l_noise) \ A'*b;

end


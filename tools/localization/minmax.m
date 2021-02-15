## usage: [LM_P] = minmax (POSES_XY, P, R)
##
## Computes the position of a landmark given a set
## of poses. A set of indices p(i) refers to the poses
## observing the landmark, sensing it at distance r(i).
## Minmax makes use of squares of side 2r built
## around the poses to localize the landmark.
## Further info: https://www.sciencedirect.com/science/article/pii/S1389128603003566
##
##  poses_xy : Poses of the robot in xy coordinates
##  p : Indices of the poses referring to current landmark
##  r : Set of range measurements from p-th pose

function [lm_p] = minmax(poses_xy, p, r)
    
    r_num = length(r);

    v1_max = [];
    v2_max = [];
    v1_min = [];
    v2_min = [];

    for k=1:r_num
        [xk, yk, rk] = deal(poses_xy(1,p(k)), poses_xy(2,p(k)), r(k));
        
        v1_max(end+1) = xk - rk;
        v2_max(end+1) = yk - rk;
        v1_min(end+1) = xk + rk;
        v2_min(end+1) = yk + rk;

    endfor

    p1 = [max(v1_max); max(v2_max)];
    p2 = [min(v1_min); min(v2_min)];

    lm_p = [mean([p1(1), p2(1)]); mean([p1(2), p2(2)])];

end
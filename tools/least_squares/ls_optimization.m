source 'tools/utilities/geometry_helpers_2d.m'

## usage: [XR, XL, CHI_STATS, NUM_INLIERS] = LS (XR, XL, Z, ASSOCIATIONS, POSES_NUM, LMS_NUM, ITERATIONS_NUM, DAMPING, KERNEL_THRESHOLD)
##
## Iterative Least Squares optimization 
## algorithm for manifolds.
##
##  XR : Robot poses in SE2
##  XL : Landmarks positions in R2
##  Z  : Range observations in R
##  associations : Matrix of associated poses indices and landmarks indices
##  poses_num : Number of robot poses
##  lms_num : Number of landmarks
##  iterations_num : Number of iterations for LS optimization
##  damping : Damping factor of Hessian matrix
##  kernel_threshold : Error threshold for robust kernel
##
##  Returns
##
##  XR : Optimized robot poses in SE2
##  XL : Optimized landmarks positions in R2
##  chi_stats  : Chi evolution over the iterations
##  num_inliers : Number of inliers over the iterations

global pose_dim = 3; % Pose dimension [x, y, theta]
global lm_dim = 2; % Landmark dimension [x, y]


function [XR, XL, chi_stats, num_inliers] = LS(XR, XL, Z, associations, poses_num, lms_num, iterations_num, damping, kernel_threshold)
    global pose_dim;
    global lm_dim;

    chi_stats   = zeros(1, iterations_num);
    num_inliers = zeros(1, iterations_num);

    # Size of the linear system
    system_size = pose_dim*poses_num + lm_dim*lms_num; 

    for i=1:iterations_num
        XR(:,:,247);
        H = zeros(system_size, system_size);
        b = zeros(system_size, 1);
        chi_stats(i) = 0;

        for m=1:length(Z)
            pose_index     = associations(1, m);
            landmark_index = associations(2, m);

            z = Z(:, m);
            
            Xr = XR(:, :, pose_index);
            Xl = XL(:, landmark_index);
            
            [e,Jr,Jl] = errorAndJacobian(Xr, Xl, z);
            chi = e' * e;

            if chi > kernel_threshold
                e *= sqrt(kernel_threshold/chi);
                chi = kernel_threshold;
            else
                num_inliers(i)++;
            endif
            chi_stats(i) += chi;

            Hrr = Jr'*Jr;
            Hrl = Jr'*Jl;
            Hll = Jl'*Jl;
            
            br  = Jr'*e;
            bl  = Jl'*e;

            [p_matrix_idx, l_matrix_idx] = matrix_index(pose_index, landmark_index, poses_num, lms_num);

            H(p_matrix_idx:p_matrix_idx+pose_dim-1,
            p_matrix_idx:p_matrix_idx+pose_dim-1) += Hrr;

            H(p_matrix_idx:p_matrix_idx+pose_dim-1,
            l_matrix_idx:l_matrix_idx+lm_dim-1) += Hrl;

            H(l_matrix_idx:l_matrix_idx+lm_dim-1,
            l_matrix_idx:l_matrix_idx+lm_dim-1) += Hll;

            H(l_matrix_idx:l_matrix_idx+lm_dim-1,
            p_matrix_idx:p_matrix_idx+pose_dim-1) += Hrl';

            b(p_matrix_idx:p_matrix_idx+pose_dim-1) += br;
            b(l_matrix_idx:l_matrix_idx+lm_dim-1) += bl;

        endfor 

        H += eye(system_size)*damping;
        dx = zeros(system_size, 1);

        dx(pose_dim+1:end) = -(H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end, 1));
        [XR, XL] = boxPlus(XR, XL, poses_num, lms_num, dx);
        
    endfor

end


% Computes error and Jacobian
function [e, Jr, Jl] = errorAndJacobian(Xr, Xl, z)
   R = Xr(1:2,1:2);
   t = Xr(1:2,3);

   p_hat = R' * (Xl-t);
   z_hat = norm(p_hat);
   e = z_hat - z;
	 
   Jr = zeros(1,3);
   J_icp = zeros(2,3);
   J_icp(1:2,1:2) = -R';
	 
   J_icp(1:2,3) = R' * [0 1;-1 0] * Xl;
   Jr = (1/norm(p_hat)) * p_hat' * J_icp;
   Jl= (1/norm(p_hat)) * p_hat' * R';
	 
end


% Retrieves indices of poses/landmarks in the Hessian matrix
function [p_matrix_idx, l_matrix_idx] = matrix_index(p_index, lm_index, num_poses, lms_num)
    global pose_dim;
    global lm_dim;
    
    if p_index > num_poses || p_index <= 0
        p_matrix_idx = -1;
    else
        p_matrix_idx = 1 + (p_index-1)*pose_dim;
    endif


    if lm_index > lms_num || lm_index <= 0
        l_matrix_idx = -1;
    else
        l_matrix_idx = 1 + (num_poses)*pose_dim + (lm_index-1)*lm_dim;
    endif
    
end


% Box plus operator for the pertubation
function [XR, XL] = boxPlus(XR, XL, poses_num, lms_num, dx)
    global pose_dim;
    global lm_dim;

    for pose_index=1:poses_num
        [p_matrix_idx, _] = matrix_index(pose_index, 0, poses_num, lms_num);
        dxr = dx(p_matrix_idx:p_matrix_idx+pose_dim-1);
        XR(:,:,pose_index) = v2t(dxr)*XR(:,:,pose_index);

        if pose_index==247
            dxr;
        end
    endfor

    for landmark_index=1:lms_num 
        [_, l_matrix_idx] = matrix_index(0, landmark_index, poses_num, lms_num);
        dxl = dx(l_matrix_idx:l_matrix_idx+lm_dim-1, :);
        XL(:,landmark_index) += dxl;
    endfor
end
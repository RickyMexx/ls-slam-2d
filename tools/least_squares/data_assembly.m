source 'tools/utilities/geometry_helpers_2d.m'


% Computes ground truth and initial guess SE2 poses
function [Xr, Xr_gt, poses_ig_xy, poses_gt_xy] = poses_SE2(poses_ig, poses_gt, poses_num)
    Xr    = zeros(3, 3, poses_num);
    Xr_gt = zeros(3, 3, poses_num);

    poses_ig_xy = zeros(2, poses_num);
    poses_gt_xy = zeros(2, poses_num);

    for i=1:poses_num
        x  = poses_ig(i).x;
        y  = poses_ig(i).y;
        th = poses_ig(i).theta;

        x_gt  = poses_gt(i).x;
        y_gt  = poses_gt(i).y;
        th_gt = poses_gt(i).theta;

        Xr(:,:,i)    = v2t([x y th]);
        Xr_gt(:,:,i) = v2t([x_gt y_gt th_gt]);
        
        poses_ig_xy(:, i) = [x; y];
        poses_gt_xy(:, i) = [x_gt; y_gt];
    endfor
end


% Computes ground truth and initial guess SE2 poses with odometry
function [Xr_ig_odom, poses_ig_odom_xy] = poses_odom_SE2(poses_ig, transitions_ig, poses_num, trs_num)
    Xr_ig_odom = zeros(3, 3, poses_num);
    poses_ig_odom_xy = zeros(2, poses_num);
    
    % First pose is fixed (same for xy) -> Prior
    Xr_ig_odom(:,:,1)      = v2t([poses_ig(1).x poses_ig(1).y poses_ig(1).theta]);
    poses_ig_odom_xy(:, 1) = [poses_ig(1).x; poses_ig(1).y];
    
    for i=1:trs_num
        tr = transitions_ig(i).v;   
        u_x = tr(1);
        u_theta = tr(3); 
        
        % Actual pose of the robot
        pose_act = Xr_ig_odom(:,:,i);

        % Compute pose displacement and get next pose
        pose_d = v2t([u_x 0 u_theta]);
        pose_next = pose_act * pose_d; 
        Xr_ig_odom(:,:,i+1) = pose_next; 

        % Fill xy poses vector
        poses_ig_odom_xy(:, i+1) = pose_next(1:2, 3);
    endfor

end


% Computes ground truth and initial guess R3 poses
function [poses_ig_xyt, poses_gt_xyt] = poses_R3(poses_ig, poses_gt, poses_num)

    poses_ig_xyt = zeros(3, poses_num);
    poses_gt_xyt = zeros(3, poses_num);

    for i=1:poses_num
        x  = poses_ig(i).x;
        y  = poses_ig(i).y;
        th = poses_ig(i).theta;

        x_gt  = poses_gt(i).x;
        y_gt  = poses_gt(i).y;
        th_gt = poses_gt(i).theta;
        
        poses_ig_xyt(:, i) = [x; y; th];
        poses_gt_xyt(:, i) = [x_gt; y_gt; th_gt];
    endfor
end


% Computes landmarks in R2
function [lms_xy] = lms_R2(lms, lm_indices, lms_num)
    lms_xy = zeros(2, lms_num);

    for i=1:lms_num
        lm = lms(i);
        lm_idx = lm_indices(lm.id);
        lms_xy(:,lm_idx) = [lm.x_pose; lm.y_pose];
    endfor
end
close all
clear
clc

addpath 'tools/g2o_wrapper'
addpath 'tools/localization'
addpath 'tools/least_squares'
source 'tools/least_squares/data_assembly.m'
source 'tools/least_squares/ls_optimization.m'
source 'tools/visualization/plotter.m'


% ------------ Parameters ------------ %

dataset = "03-RangeOnlySLAM"; % Choose which dataset to use (must contain init guess + ground truth files)
data_path = "data"; % Data folder path
plots_path = "plots"; % Plots folder path

% Standard initial guess and ground truth file names (change if needed)
ig_name = "slam2d_range_only_initial_guess.g2o";
gt_name = "slam2d_range_only_ground_truth.g2o";

ig_algorithm = "lateration"; % Pick a localization algorithm {lateration,minmax} for the initial guess of LMs
variance_threshold = 0.6; % We choose observations with a variance higher or equal than this parameter when there are too many 
max_range_obs = 6; % Take up to max_range_obs observations if there are too many and their variance is lower than variance_threshold
l_noise = 9e-3; % Noise parameter for lateration algorithm 

iterations_num = 20; % Number of iterations for LS optimization
damping = 0.15; % Damping factor of Hessian matrix
kernel_threshold = 1.0; % Error threshold for robust kernel

tf_vec = [-0.2 -1.8 0.22]; % Transformation vector for a better visualization of the result (simulated calibration)

% ------------------------------------ %


% ----------- Prepare data ----------- %

% Dataset loading 
printf(cstrcat("\n## Loading ", dataset, " dataset ##\n\n"));
[_, poses_ig, transitions_ig, observations_ig] = loadG2o(fullfile(data_path, dataset, ig_name));
[landmarks_gt, poses_gt, transitions_gt, observations_gt] = loadG2o(fullfile(data_path, dataset, gt_name));
printf("\n");

% Retrieve cardinality of the problem
lms_num   = length(landmarks_gt);
poses_num = length(poses_gt);
trs_num   = length(transitions_gt);
obs_num   = length(observations_gt);

% Assemble robot poses in SE(2)
[Xr_ig, Xr_gt, poses_ig_xy, poses_gt_xy] = poses_SE2(poses_ig, poses_gt, poses_num);
% Compute poses initial guess using odometry
[Xr_ig_odom, poses_ig_odom_xy] = poses_odom_SE2(poses_ig, transitions_ig, poses_num, trs_num);


% ----------- Initial guess ----------- %

printf("## Computing initial guess of landmarks ##\n");

% Compute initial guess of landmarks position using range observations
[landmarks_ig_xy, lm_indices, lm_ids, lm_indices_all, lm_ids_all, avg_res, associations, Z] = init_guess(poses_ig, observations_ig, poses_ig_odom_xy,
                                                                                                         poses_num, obs_num, lms_num,
                                                                                                         ig_algorithm, l_noise, 
                                                                                                         max_range_obs, variance_threshold);
% Number of landmarks considered
real_lms_num = length(landmarks_ig_xy);

% Assemble landmarks GT in R2
[landmarks_gt_xy] = lms_R2(landmarks_gt, lm_indices_all, lms_num);

% Evaluate landmarks intial guess
mse_pre_opt = eval_guess(landmarks_ig_xy, landmarks_gt_xy,
                         lm_indices_all, lm_ids, real_lms_num);

printf("# Average residue of %s algorithm: %f \n", ig_algorithm, avg_res);
printf("# MSE of LMs initial guess: %f \n\n", mse_pre_opt);


% ----------- Least Squares ----------- %

printf("## Starting LS optimization ##\n");

% Start Least Squares optimization
[Xr, Xl, chi_stats, num_inliers] = LS(Xr_ig_odom, landmarks_ig_xy, 
                                      Z, associations,
                                      poses_num, real_lms_num, 
                                      iterations_num, 
                                      damping, kernel_threshold);

% Evaluate optimized landmarks guess
mse_post_opt = eval_guess(Xl, landmarks_gt_xy,
                          lm_indices_all, lm_ids, real_lms_num);

printf("# MSE of LMs optimized guess: %f \n\n", mse_post_opt);

% --------------- Plots --------------- %

printf("## Generating plots and saving them in /%s folder ##\n\n", plots_path);

% Map plot: Lanmarks IG, OPT, GT
plot_map(landmarks_ig_xy, Xl, landmarks_gt_xy, plots_path, ig_algorithm, dataset);

% Trajectory plot: Poses IG, OPT, GT
plot_traj(Xr_ig_odom, Xr, Xr_gt, poses_num, plots_path, ig_algorithm, dataset);

% Trajectory plot w/ simulated odometry calibration: Poses IG, OPT, GT
plot_cal_traj(Xr_ig_odom, Xr, Xr_gt, poses_num, tf_vec, plots_path, ig_algorithm, dataset);

%pause(200);
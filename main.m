close all
clear
clc

addpath 'tools/g2o_wrapper'


% ------------ Parameters ------------ %

dataset = "03-RangeOnlySLAM"; % Choose which dataset to use (must contain init guess + ground truth files)

% Standard initial guess and ground truth file names (change if needed)
ig_name = "slam2d_range_only_initial_guess.g2o";
gt_name = "slam2d_range_only_ground_truth.g2o";


% ------------------------------------ %


% Dataset loading %

printf(cstrcat("## Loading ", dataset, " dataset ##\n"))
[_, poses_ig, transitions_ig, observations_ig] = loadG2o(fullfile('data', dataset, ig_name));
[landmarks_gt, poses_gt, transitions_gt, observations_gt] = loadG2o(fullfile('data', dataset, gt_name));


% TO-DO %


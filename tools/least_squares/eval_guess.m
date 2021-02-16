## usage: [MSE] = eval_guess (LMS_IG, LMS_GT, LM_INDICES, LM_IDS, LMS_NUM)
##
## Computes the Mean Squared Error between
## the ground truth landmarks lms_gt and the
## initial guess lms_ig.
##
##  lms_ig : Initial guess landmarks in xy coordinates
##  lms_gt : Ground truth landmarks in xy coordinates
##  lm_indices : Vector of landmarks indices indexed with their ids
##  lm_ids : Vector of landmarks ids indexed with their index
##  lms_num: Number of landmarks 

function [mse] = eval_guess(lms_ig, lms_gt, lm_indices, lm_ids, lms_num)
    sq_err = 0;
    
    for i=1:lms_num
        lm_id  = lm_ids(i);
        lm_idx = lm_indices(lm_id);

        % Retrieve gt and ig
        lm_gt = lms_gt(:, lm_idx);
        lm_ig = lms_ig(:, i);
        
        % Compute euclidean distance
        sq_err += norm(lm_gt - lm_ig);
    endfor

    mse = sq_err / lms_num;

end
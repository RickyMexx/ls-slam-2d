## usage: [...] = init_guess (POSES_IG, OBSERVATIONS_IG, POSES_XY, POSES_NUM, OBS_NUM, LMS_NUM, 
##                            ALGORITHM, L_NOISE, MAX_RANGE_OBS, VARIANCE_THRESHOLD)
##
## Computes the initial guess of the landmarks
## given a localization algorithm. This procedure
## returns only a subset of the total landmarks.
## Indeed, landmarks with few observations or low 
## observations variance are excluded.
##
##  poses_ig : Initial guess poses
##  observations_ig : Initial guess observations
##  poses_xy : Initial guess poses in xy coordinates
##  poses_num : Number of robot poses
##  obs_num  : Number of observations
##  lms_num  : Number of landmarks
##  algorithm : Localization algorithm to guess lms position
##  l_noise  : Noise parameter for lateration algorithm 
##  max_range_obs : Maximum number of observations considered when having low variance
##  variance_threshold : Threshold of the variance when many observations are present
##
##  Returns
##
##  landmarks_ig : Landmarks in xy coordinates
##  lm_indices : Vector of chosen landmarks indices indexed with their ids 
##  lm_ids  : Vector of chosen landmarks ids indexed with their index
##  lm_indices_all : Vector of all landmarks indices indexed with their ids 
##  lm_ids_all : Vector of all landmarks ids indexed with their index
##  avg_res : Average residue of the localization algorithm
##  associations : Matrix of associated poses indices and landmarks indices
##  Z : Vector of range observations related to the associations

function [landmarks_ig, lm_indices, lm_ids, lm_indices_all, lm_ids_all, avg_res, associations, Z] = init_guess(poses_ig, observations_ig, poses_xy, 
                                                                                                    poses_num, obs_num, lms_num, algorithm, 
                                                                                                    l_noise, max_range_obs, variance_threshold)
    
    % Vectors to map landmarks indices with
    % their ids.
    lm_indices_all = ones(10000, 1) * -1;
    lm_ids_all = ones(lms_num, 1) * -1;

    % Pair landmarks and their observers in a matrix. Access it in o(1) 
    observed_lms = ones(lms_num, poses_num) * -1;
    idx = 1;

    % Matrix construction
    first_pose_id = poses_ig(1).id;
    for i=1:obs_num
        obs = observations_ig(i);
        pose_id = observations_ig(i).pose_id;
        p_idx = pose_id - first_pose_id + 1;
        
        for j=1:length(obs.observation)
            lm = obs.observation(j).id;
            rn = obs.observation(j).range;
    
            % Check lm index
            if lm_indices_all(lm) == -1
                lm_indices_all(lm) = idx;
                lm_ids_all(idx) = lm;
                idx += 1;
            endif
            
            observed_lms(lm_indices_all(lm), p_idx) = rn;

        endfor
    endfor


    landmarks_ig = zeros(2, lms_num);
    real_lms_num = 1;
    real_indices = [];
    tot_res = [];

    lm_indices = ones(10000, 1) * -1;
    lm_ids = ones(lms_num, 1) * -1;
    idx = 1;

    for i=1:lms_num

        r = []; % Range values 
        p = []; % Poses indices

        % Retrieve range observations
        for j=1:poses_num
            range_obs = observed_lms(i, j);
            if range_obs != -1
                r(end+1) = observed_lms(i, j);
                p(end+1) = j;
            endif
        endfor

        % Update real indices and ids arrays
        lm_id = lm_ids_all(i);
        lm_indices(lm_id) = idx;
        lm_ids(idx) = lm_id;

        % Check whether i-th landmark is a valid one
        variance = var(r);

        if length(r) > max_range_obs && variance < variance_threshold
            r_s  = []; % Range values sampled 
            p_s  = []; % Poses indices sampled

            % Sample every step_s steps
            step_s = floor(length(r) / max_range_obs);
            idx_s = 1;
            for i=1:max_range_obs
                idxs(end+1) = idx_s;
                r_s(end+1)  = r(idx_s);
                p_s(end+1)  = p(idx_s);

                idx_s += step_s;
            endfor
        % We need at least 3 range observations
        elseif length(r) < 3 
            continue;
        else    
            r_s = r;
            p_s = p;
        end
        
        % If the landmark is valid pick an algorithm
        % and estimate its position
        switch(algorithm)
			case "minmax"
        		lm_pos = minmax(poses_xy, p_s, r_s);
			case "lateration"
        		lm_pos = lateration(poses_xy, p_s, r_s, l_noise);
			otherwise
				disp('Choose a valid algorithm.');
		end

        landmarks_ig(:,idx) = lm_pos;
        idx++;
        
        tot_res(end+1) = compute_residue(poses_xy, p, r, lm_pos);

    endfor

    real_lms_num = idx - 1;
    landmarks_ig = landmarks_ig(:,1:real_lms_num);
    avg_res = abs(mean(tot_res));


    % Init associations and Z to the largest scenario (every pose observes every landmark)
    associations = zeros(2, poses_num*real_lms_num);
    Z = zeros(1, poses_num*real_lms_num);
    z_idx = 1;

    % Fill associations and Z
    for i=1:poses_num
        for j=1:real_lms_num
            lm_id = lm_ids(j);
            range_val = observed_lms(lm_indices_all(lm_id), i);

            if range_val != -1
                associations(:,z_idx) = [i; j];
                Z(:,z_idx) = range_val;
                
                z_idx++;
            end
        endfor
        
    endfor
    
    associations = associations(:, 1:z_idx-1);
    Z = Z(:, 1:z_idx-1);

end


% Computes residue of the localization algorithm
function res = compute_residue(poses_xy, p, r, lm)
    res = [];
    r_num = length(r);

    for k=1:r_num
        [xk, yk, rk]  = deal(poses_xy(1,p(k)), poses_xy(2,p(k)), r(k));
        res(end+1) = sqrt((xk-lm(1))^2 + (yk-lm(2))^2) - rk;
    endfor

    res = mean(res);
end
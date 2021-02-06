function out = drawObservations(pose, observations)

	N = length(observations.observation);
	l = 1.5;

	if N > 0
	
	  #determine highlight mode
		mode = 'points';
		if isfield(observations.observation(1),'bearing')
			mode = 'bearing';
		end
		
		hold on;
		
		#for each observation
		for i=1:N
			current_observation = observations.observation(i);
			
			#default: points
			if strcmp(mode, 'points')
			
			  #draw points - with pointers towards them
				land_obs = [current_observation.x_pose; current_observation.y_pose; 0];		
				land_abs_pose = t2v(v2t(pose)*v2t(land_obs));
				current_observation.x_pose = land_abs_pose(1);
				current_observation.y_pose = land_abs_pose(2);
        drawShape('circle', [land_abs_pose(1), land_abs_pose(2), 0.2], 'fill', 'b');

			else
			
			  #draw measured bearing
				land_obs = current_observation.bearing;
				incr_y = sin(land_obs)*l;
				incr_x = cos(land_obs)*l;
				ray_x = pose(1) + incr_x*cos(pose(3)) - incr_y*sin(pose(3));
				ray_y = pose(2) + incr_y*cos(pose(3)) + incr_x*sin(pose(3));
				plot([pose(1) ray_x] , [pose(2) ray_y], 'b', 'linewidth', 1.5);
			endif
		endfor		
	endif
endfunction


function out = plotState(varargin)
  landmarks    = varargin{1};
  pose         = varargin{2};
  if (nargin > 2)
    covariance   = varargin{3};
    observations = varargin{4};
  endif

  #check if we have additional input
  if (nargin > 4)
    trajectory = varargin{5};
  endif

	robot_pose = pose(1:3);
	state_dim  = size(pose,1);
	map_size   = (state_dim - 3)/2; #number of landmarks in the state (if any)

	if(nargin == 2) %init step
    hold off;
		drawLandmarks(landmarks);
		drawRobot(robot_pose, zeros(3));
	else
		hold off;

    #draw robot trajectory
    if (nargin > 4)
      drawTrajectoryXY(trajectory);
    endif

    #highlight current observations
    drawObservations(robot_pose, observations);

    #draw landmarks
		drawLandmarks(landmarks);

		#plot robot covariance
    plotcov2d(robot_pose(1),robot_pose(2),covariance(1:2,1:2),'k', 1);

		#plot landmark covariances (if any)
		for i=4:2:2*(map_size+1)
		  plotcov2d(pose(i,1), pose(i+1,1), covariance(i:i+1,i:i+1),'r',1);
		endfor

    #draw robot pose
    drawRobot(robot_pose, covariance);

	endif
  axis([-11, 11, -11, 11]);
	drawnow;
endfunction

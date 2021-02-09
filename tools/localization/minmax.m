## usage: [LANDMARKS, RESIDUES] = minmax (COMMAND, ARGS)
##
## Start a subprocess with two-way communication.  COMMAND
## specifies the name of the command to start.  ARGS is an
## array of strings containing options for COMMAND.  IN and
## OUT are the file ids of the input and streams for the
## subprocess, and PID is the process id of the subprocess,
## or -1 if COMMAND could not be executed.
##
## Example:
##
##  [in, out, pid] = popen2 ("sort", "-nr");
##  fputs (in, "these\nare\nsome\nstrings\n");
##  fclose (in);
##  while (ischar (s = fgets (out)))
##    fputs (stdout, s);
##  endwhile
##  fclose (out);

function [x, y] = minmax(poses_xy, p, r)
    
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

    x = lm_p(1);
    y = lm_p(2);

end
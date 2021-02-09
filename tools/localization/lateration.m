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

function [x, y] = lateration(poses_xy, p, r)
    r_num = length(r);

    [xn, yn, rn] = deal(poses_xy(1,p(r_num)), poses_xy(2,p(r_num)), r(r_num));

    A = zeros(r_num-1, 2);
    b = zeros(r_num-1, 1);

    for k=1:r_num-1
        [xk, yk, rk] = deal(poses_xy(1,p(k)), poses_xy(2,p(k)), r(k));
        
        A(k,:) = [2*(xk - xn) 2*(yk - yn)];
        b(k,:) = [xk^2-xn^2 + yk^2-yn^2 + rn^2-rk^2];
    endfor

    lm_p = A'*A \ A'*b;

    x = lm_p(1);
    y = lm_p(2);

end
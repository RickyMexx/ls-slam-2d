#functor that computes the numeric jacobian
# f: function pointer, eg @norm
# x: evaluation point
# order : set of indices for the evauated variables
# eg:
#
#J_numeric(@normalize, [0.1, 0.2, 0.3]', [1,3])
# evaluates the jacobian of the normalize(p) function
# in the point [0.1, 0.2, 0.3]'
# the first column is the derivative w.r.t. x (index 1)
# the second column is the derivative w.r.t. z (index 3)

function J = J_numeric(f,x,order)
  dx=1e-6;
  if (size(order)==0)
    order=1:size(x);
  endif;
  diff_size=size(order, 2);
  y=f(x);
  y_size=size(y);
  J=zeros(y_size, diff_size);
  pert=x;
  pert*=0;
  for (n = 1:diff_size)
    idx=order(n);
    pert(idx)=dx;
    J(:,n)=f(x+pert)-f(x-pert);
    pert(idx)=0;
  endfor;
  J/=(2*dx);
endfunction

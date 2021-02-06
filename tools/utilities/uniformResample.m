%uniform resampling
%generates an array of indices of size num_desired_samples 
%the probability that the index i occurs in the output
%is weights[i]
%inputs:
% weights: the non normalized weights
% num_desired_samples: the number od samples generated
%outputs:
% sampled_indices: the indices sampled with probability weights[i]

function sampled_indices=uniformResample(weights, num_desired_samples)
  %normalize the weights (if they are not normalized)
  normalizer=1./sum(weights);
  %resize the indices
  sampled_indices=zeros(num_desired_samples,1);
  step=1./num_desired_samples;
  
  y0=rand()*step;     %sample between 0 and 1/num_desired_samples
  yi=y0;              %value of the sample on the y space           
  cumulative =0;      %this is our running cumulative distribution
  sample_index=1;     %the index of output where we write the sampled idx
  for (weight_index=1:size(weights,1))
      cumulative += normalizer*weights(weight_index); %update cumulative
      % fill with current_weight_index 
      % until the cumulative does not become larger than yi
      while (cumulative>yi)
	    sampled_indices(sample_index)=weight_index;
	    sample_index++;
	    yi+=step;
      endwhile
  endfor
endfunction

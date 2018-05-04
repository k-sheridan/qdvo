function [weight] = huberLossFn(chi_abs)
%computes the huber weight for this error
HUBER_THRESH = 1e-6;

if(chi_abs <= HUBER_THRESH)
   weight = 1; 
else
    weight = HUBER_THRESH/chi_abs;
end

end


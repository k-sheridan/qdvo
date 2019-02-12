classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties
        residualJacobianBuffer = {}; % stores all of the jacobians and residuals for the update. {residual, JacobianContainer}
        
        
    end
    
    methods
        
    end
end


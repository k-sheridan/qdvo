classdef PriorErrorTerm < handle
    %PRIORERRORTERM stores the approximmated marginalized information in a
    %quadratic error term. It can be though of as a prior. 
    %       error(x) = 1/2*(x-x0)'*A*(x-x0) + (x-x0)'*b
    % In the case of a variable with multiplicative updates. We fi its
    % manifold at the point of marginalization.
    
    properties
        A = []; % this can be thought of as an inverse covariance matrix. 
        b = []; % this can be thought of as a mean of a gaussian on manifold.
        dx0 = []; % this term is constantly added to until the final iteration, where the prior is reconstructed.
        indexHandler; % this is used to describe the variable order in the prior. It is also used to correct the system and change its order.
    end
    
    methods
        function obj = PriorErrorTerm()
            
        end
        
        % This simply initializes the error term as a uniformly uncertain
        % gaussian.
        function [] = initializePriorUncertain(obj, indexHandler, inverseUncertainty)
            obj.indexHandler = indexHandler;
            varVec = ones(1, obj.indexHandler.dimensions()) * inverseUncertainty;
            
            obj.dx0 = zeros(obj.indexHandler.dimensions(), 1);
            obj.b = zeros(obj.indexHandler.dimensions(), 1);
            obj.A = diag(varVec);
        end
        
        % This function will set the new prior. (this is typically used
        % after marginalizing variables).
        % dx0 will be set to 0!
        function [] = initialize(obj, indexHandler, A, b)
            % check that A and b have the same dimensions as the variables.
            if length(b) ~= indexHandler.dimensions()
                error('b dimensions wrong');
            end
            [m,n] = size(A);
            if n ~= indexHandler.dimensions() || m ~= n
                error('A dimensions wrong');
            end
            
            obj.indexHandler = indexHandler;
            obj.A = A;
            obj.b = b;
            obj.dx0 = zeros(obj.indexHandler.dimensions(), 1);
        end
        
    end
end


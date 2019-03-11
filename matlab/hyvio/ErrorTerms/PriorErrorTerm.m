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
            obj.A = [];
            obj.b = [];
            obj.dx0 = [];
            obj.indexHandler = IndexHandler();
            obj.indexHandler.reset();
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
        
        %% add new variables to the prior error term.
        function [] = addLandmark(obj, parentFrameID, landmarkID, informationMatrix)
            key = obj.indexHandler.landmarkKey(parentFrameID, landmarkID);
            
            if ~obj.indexHandler.hasKey(key)
                % now we know that this variable will be added to the end
                % of the variable order.
                [m,n] = size(informationMatrix);
                if m ~= n
                    error('information matrices must be square!')
                end
                
                if m ~= 1
                    error('information matrix not the correct size')
                end
            
                obj.indexHandler.addLandmark(parentFrameID, landmarkID);
            
                indices = obj.indexHandler.getLandmarkIndices(parentFrameID, landmarkID);
                
                obj.A = padarray(obj.A,[m,m],0,'post');
                obj.A(indices, indices) = informationMatrix;
                
                obj.b = [obj.b; zeros(m, 1)];
                obj.dx0 = [obj.dx0; zeros(m, 1)];
            end
        end
        
        function [] = addImustate(obj, frameId, informationMatrix)
            key = obj.indexHandler.imustateKey(frameId);
            
            if ~obj.indexHandler.hasKey(key)
                % now we know that this variable will be added to the end
                % of the variable order.
                [m,n] = size(informationMatrix);
                if m ~= n
                    error('information matrices must be square!')
                end
                
                if m ~= 15
                    error('information matrix not the correct size')
                end
            
                obj.indexHandler.addImustate(frameId);
            
                indices = obj.indexHandler.getImustateIndices(frameId);
                
                obj.A = padarray(obj.A,[m,m],0,'post');
                obj.A(indices, indices) = informationMatrix;
                
                obj.b = [obj.b; zeros(m, 1)];
                obj.dx0 = [obj.dx0; zeros(m, 1)];
            end
        end
        
        function [] = addExtrinsic(obj, key, informationMatrix)
            if ~obj.indexHandler.hasKey(key)
                % now we know that this variable will be added to the end
                % of the variable order.
                [m,n] = size(informationMatrix);
                if m ~= n
                    error('information matrices must be square!')
                end
            
                obj.indexHandler.addExtrinsic(key, m);
            
                indices = obj.indexHandler.getExtrinsicIndices(key);
                
                obj.A = padarray(obj.A,[m,m],0,'post');
                obj.A(indices, indices) = informationMatrix;
                
                obj.b = [obj.b; zeros(m, 1)];
                obj.dx0 = [obj.dx0; zeros(m, 1)];
            end
        end
        
    end
end


classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties (Access = private)
        constraintBuffer = {}; % stores all of the jacobians and residuals for the update. struct(residual, information, JacobianContainer)
        errorTermContainer = {}; % stores all error terms for this optimizer.
        
        indexHandler; % this handles the tricky step of knowing what variable is associated to whhich index. This also handles the keys.
    end
    
    methods
        function [obj] = Optimizer()
            obj.indexHandler = IndexHandler();
            obj.errorTermContainer = {};
        end
        
        function [] = addErrorTerm(obj, errorTerm)
            % adds a generic error term to the
            obj.errorTermContainer{end+1} = errorTerm;
        end
        
        
        % using the error terms currently in the optimizer, optimize the
        % graph.
        function [graph] = optimize(obj, graph)
            % First, compute the residuals.
            obj.computeResiduals(graph);
            
            % Setup the indexhandler for this optimization.
            obj.initializeIndexHandler();
            
            % TODO make the prior error term compatible with the current
            % variable order.
            
            % perform gauss newton optimization.
            
        end
        
        function [] = computeResiduals(obj, graph)
            % empty the constraint buffer to be refilled again.
            obj.constraintBuffer = {};
            
            for idx = (1:length(obj.errorTermContainer))
                [r, information, J] = obj.errorTermContainer{idx}.computeResidual(graph);
                
                s = struct('residual', r, 'information', information, 'jacobians', J);
                
                obj.constraintBuffer{end+1} = s;
            end
        end
        
        % looks at the variables to be optimized (in constraint buffer), and makes a mapping
        % between their id and indices.
        function [] = initializeIndexHandler(obj)
            % reset the current IndexHandler
            obj.indexHandler = IndexHandler();
            
            % sweep for the extrinsics.
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.extrinsicJacobians))
                    j = jc{innerIdx};
                    key = j{1};
                    obj.indexHandler.addExtrinsic(key); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the landmarks
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.landmarkJacobians))
                    j = jc{innerIdx};
                    pidx = j{1};
                    lidx = j{2};
                    obj.indexHandler.addLandmark(pidx, lidx); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the imustates
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.imustateJacobians))
                    j = jc{innerIdx};
                    fidx = j{1};
                    obj.indexHandler.addImustate(fidx); % this checks if the landmark has already been added, and adds it.
                end
            end
        end
    end
end


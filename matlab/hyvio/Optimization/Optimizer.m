classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties (Access = private)
        constraintBuffer = {}; % stores all of the jacobians and residuals for the update. struct(residual, information, jacobians)
        errorTermContainer = {}; % stores all error terms for this optimizer.
        
        A = [];
        b = [];
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
        
        function [] = clearErrorTerms(obj)
            obj.errorTermContainer = {};
        end
        
        function [num] = numberOfErrorTerms(obj)
            num = length(obj.errorTermContainer);
        end
        
        % using the error terms currently in the optimizer, optimize the
        % graph.
        function [graph] = optimize(obj, graph)
            % First, compute the residuals.
            obj.constraintBuffer = cell(1, length(obj.errorTermContainer));
            obj.computeResiduals(graph);
            
            length(obj.constraintBuffer)
            
            % Setup the indexhandler for this optimization.
            obj.initializeIndexHandler();
            
            % TODO make the prior error term compatible with the current
            % variable order.
            
            
            
            % perform gauss newton optimization.
            niter = 10;
            for it = (1:niter)
                % reset A, and b;
                obj.A = zeros(obj.indexHandler.dimensions());
                obj.b = zeros(obj.indexHandler.dimensions(), 1);
                % build A and b
                for c = obj.constraintBuffer
                    J = obj.createConstraintJacobian(c{1}.jacobians, length(c{1}.residual));
                    sW = sparse(c{1}.information);
                    obj.A = obj.A + J'*sW*J;
                    obj.b = obj.b + J'*sW*c{1}.residual;
                end
                
                % after update, recompute the residuals
                tic
                obj.computeResiduals(graph);
                toc
                
            end
            obj.b
            image(obj.A)
        end
        
        function [] = computeResiduals(obj, graph)
            
            for idx = (1:length(obj.errorTermContainer))
                [r, information, J] = obj.errorTermContainer{idx}.computeResidual(graph);
                
                s = struct('residual', r, 'information', information, 'jacobians', J);
                
                obj.constraintBuffer{idx} = s;
            end
            
        end
        
        % creates a sparse matrix, J, such that J*dx ~ r
        function [J] = createConstraintJacobian(obj, jacobianContainer, residualDim)
            % create an empty J first.
            J = zeros(residualDim, obj.indexHandler.dimensions());
            
            % landmarks
            for idx = (1:length(jacobianContainer.landmarkJacobians))
                
                pid = jacobianContainer.landmarkJacobians{idx}{1};
                lid = jacobianContainer.landmarkJacobians{idx}{2};
                jac = jacobianContainer.landmarkJacobians{idx}{3};
                
                J(1:residualDim, obj.indexHandler.getLandmarkIndices(pid, lid)) = jac;
            end
            
            % imustates
            for idx = (1:length(jacobianContainer.imustateJacobians))
                
                fid = jacobianContainer.imustateJacobians{idx}{1};
                jac = jacobianContainer.imustateJacobians{idx}{2};
                
                J(1:residualDim, obj.indexHandler.getImustateIndices(fid)) = jac;
            end
            
            % extrinsics
            for idx = (1:length(jacobianContainer.extrinsicJacobians))
                
                key = jacobianContainer.extrinsicJacobians{idx}{1};
                jac = jacobianContainer.extrinsicJacobians{idx}{2};
                
                J(1:residualDim, obj.indexHandler.getExtrinsicIndices(key)) = jac;
            end
            
            sJ = sparse(J);
        end
        
        % looks at the variables to be optimized (in constraint buffer), and makes a mapping
        % between their id and indices.
        function [] = initializeIndexHandler(obj)
            % reset the current IndexHandler
            obj.indexHandler.reset();
            
            % sweep for the extrinsics.
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.extrinsicJacobians))
                    j = jc.jacobians.extrinsicJacobians{innerIdx};
                    key = j{1};
                    obj.indexHandler.addExtrinsic(key); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the landmarks
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.landmarkJacobians))
                    j = jc.jacobians.landmarkJacobians{innerIdx};
                    pidx = j{1};
                    lidx = j{2};
                    obj.indexHandler.addLandmark(pidx, lidx); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the imustates
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.imustateJacobians))
                    j = jc.jacobians.imustateJacobians{innerIdx};
                    fidx = j{1};
                    obj.indexHandler.addImustate(fidx); % this checks if the landmark has already been added, and adds it.
                end
            end
            
        end
    end
end


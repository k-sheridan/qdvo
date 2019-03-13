classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties (Access = private)
        constraintBuffer = {}; % stores all of the jacobians and residuals for the update. struct(residual, information, jacobians)
        errorTermContainer = {}; % stores all error terms for this optimizer.
        
        prior; % this handles the tricky step of knowing what variable is associated to whhich index. This also handles the keys.
        % this also contains a quadratic (gaussian) prior error term.
        
        A = [];
        b = [];
    end
    
    methods
        function [obj] = Optimizer()
            obj.errorTermContainer = {};
            obj.prior = PriorErrorTerm;
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
            
            % Setup the indexhandler/prior for this optimization.
            obj.initialize();
            
            % make a delta vec;
            deltaArray = [];
            avgWhiteSqErrorArray = [];
            
            % perform gauss newton optimization.
            niter = 2;
            for it = (1:niter)
                % reset A, and b;
                obj.A = zeros(obj.prior.indexHandler.dimensions());
                obj.b = zeros(obj.prior.indexHandler.dimensions(), 1);
                % build A and b
                whitenedSqError = 0;
                for c = obj.constraintBuffer
                    J = obj.createConstraintJacobian(c{1}.jacobians, length(c{1}.residual));
                    sW = sparse(c{1}.information);
                    obj.A = obj.A + J'*sW*J;
                    obj.b = obj.b + J'*sW*c{1}.residual;
                    
                    whitenedSqError = whitenedSqError + c{1}.residual'*sW*c{1}.residual;
                end
                
                % add the prior constraint
                obj.A = obj.A + obj.prior.A;
                obj.b = obj.b + (obj.prior.b - obj.prior.A*obj.prior.dx0);
                
                % Compute the average weighted squared error.
                avgWhiteSqError = whitenedSqError / length(obj.constraintBuffer);
                
                % append the errors
                avgWhiteSqErrorArray = [avgWhiteSqErrorArray, avgWhiteSqError]
                
                % check if the error has increased
                if it > 1
                    if avgWhiteSqErrorArray(end) >= avgWhiteSqErrorArray(end-1)
                        % The avg error has increased. 
                        disp('error has increased!')
                    end
                end
                
                %opts.POSDEF = true;
                opts.SYM = true;
                dx = linsolve(obj.A, obj.b, opts);
                
                % append deltas
                deltaArray = [deltaArray, dx];
                
                % apply the update.
                [graph] = obj.applyUpdate(graph, dx);
                
                graph.FrameContainer{1}.imustate.p
                graph.FrameContainer{1}.imustate.R
                graph.FrameContainer{1}.imustate.v
                
                % after update, recompute the residuals
                obj.computeResiduals(graph);
                
            end
            
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
            J = zeros(residualDim, obj.prior.indexHandler.dimensions());
            
            % landmarks
            for idx = (1:length(jacobianContainer.landmarkJacobians))
                
                pid = jacobianContainer.landmarkJacobians{idx}{1};
                lid = jacobianContainer.landmarkJacobians{idx}{2};
                jac = jacobianContainer.landmarkJacobians{idx}{3};
                
                J(1:residualDim, obj.prior.indexHandler.getLandmarkIndices(pid, lid)) = jac;
            end
            
            % imustates
            for idx = (1:length(jacobianContainer.imustateJacobians))
                
                fid = jacobianContainer.imustateJacobians{idx}{1};
                jac = jacobianContainer.imustateJacobians{idx}{2};
                
                J(1:residualDim, obj.prior.indexHandler.getImustateIndices(fid)) = jac;
            end
            
            % extrinsics
            for idx = (1:length(jacobianContainer.extrinsicJacobians))
                
                key = jacobianContainer.extrinsicJacobians{idx}{1};
                jac = jacobianContainer.extrinsicJacobians{idx}{2};
                
                J(1:residualDim, obj.prior.indexHandler.getExtrinsicIndices(key)) = jac;
            end
            
            sJ = sparse(J);
        end
        
        % looks at the variables to be optimized (in constraint buffer), and makes a mapping
        % between their id and indices.
        function [] = initialize(obj)
            % the prior error term controls/handles the variable order of
            % the optimization. However, as new variables are added to the
            % optimization, we need to add them to the prior and make a
            % spot for them.
            
            % sweep for the extrinsics.
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.extrinsicJacobians))
                    j = jc.jacobians.extrinsicJacobians{innerIdx};
                    key = j{1};
                    error('I cant handle this right now in the optimizer!')
                    obj.prior.addExtrinsic(key); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the landmarks
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.landmarkJacobians))
                    j = jc.jacobians.landmarkJacobians{innerIdx};
                    pidx = j{1};
                    lidx = j{2};
                    %TODO set this from a global setting
                    pinv = 1e-24;
                    
                    obj.prior.addLandmark(pidx, lidx, pinv); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % sweep for the imustates
            for idx = (1:length(obj.constraintBuffer))
                jc = obj.constraintBuffer{idx};
                
                for innerIdx = (1:length(jc.jacobians.imustateJacobians))
                    j = jc.jacobians.imustateJacobians{innerIdx};
                    fid = j{1};
                    
                    %TODO make this a setting
                    pinv = diag(ones(1, 15) * 1e-24);
                    if fid == 1
                        pinv(1:6, 1:6) = eye(6)*1e24; % this says that the pose of the first frame is known 100%.
                    end
                    
                    obj.prior.addImustate(fid, pinv); % this checks if the landmark has already been added, and adds it.
                end
            end
            
            % check the index handler
            obj.prior.indexHandler.checkVariables();
            
        end
        
        % updates the graph and prior error term with a generalized
        % addition operator.
        function [graph] = applyUpdate(obj, graph, dx)
            % first update the prior error term.
            obj.prior.update(dx);
            
            % Update the graph using the index handler.
            [graph] = obj.prior.indexHandler.updateGraph(graph, dx);
            
        end
    end
end


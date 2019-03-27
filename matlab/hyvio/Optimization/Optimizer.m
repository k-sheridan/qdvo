classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties %(Access = private)
        constraintBuffer = {}; % stores all of the jacobians and residuals for the update. struct(residual, information, jacobians)
        errorTermContainer = {}; % stores all error terms for this optimizer.
        
        prior; % this handles the tricky step of knowing what variable is associated to whhich index. This also handles the keys.
        % this also contains a quadratic (gaussian) prior error term.
        
        A = [];
        b = [];
        
        % for debugging and mroe
        deltaArray;
        avgWhiteSqErrorArray;
    end
    
    methods
        function [obj] = Optimizer()
            obj.errorTermContainer = {};
            obj.prior = PriorErrorTerm();
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
        
        function [prior] = getPrior(obj)
            prior = obj.prior;
        end
        
        % using the error terms currently in the optimizer, optimize the
        % graph.
        function [graph] = optimize(obj, graph)
            %assert(all(obj.prior.dx0 == 0));
            
            % First, compute the residuals.
            obj.constraintBuffer = cell(1, length(obj.errorTermContainer));
            obj.computeResiduals(graph);
            
            % Setup the indexhandler/prior for this optimization.
            obj.initialize(graph);
            
            % make a delta vec;
            obj.deltaArray = [];
            obj.avgWhiteSqErrorArray = [];
            
            lambda = 1e3;
            v = 10;
            
            % perform gauss newton optimization.
            niter = 10;
            for it = (1:niter)
                % reset A, and b;
                obj.A = zeros(obj.prior.indexHandler.dimensions());
                obj.b = zeros(obj.prior.indexHandler.dimensions(), 1);
                % build A and b
                whitenedSqError = 0;
                for c = obj.constraintBuffer
                    try
                        J = obj.createConstraintJacobian(c{1}.jacobians, length(c{1}.residual));
                    catch
                        continue;
                    end
                    sW = sparse(c{1}.information);
                    obj.A = obj.A + J'*sW*J;
                    obj.b = obj.b - J'*sW*c{1}.residual;
                    
                    whitenedSqError = whitenedSqError + c{1}.residual'*sW*c{1}.residual;
                end
                
                % add the prior constraint
                obj.A = obj.A + obj.prior.A;
                obj.b = obj.b + obj.prior.b;
                
                %A*(dx0+dx)=b => A*dx + A*dx0 = b => A*dx = b - A*dx0
                
                % Compute the average weighted squared error.
                avgWhiteSqError = whitenedSqError / length(obj.constraintBuffer);
                
                % append the errors
                obj.avgWhiteSqErrorArray = [obj.avgWhiteSqErrorArray, avgWhiteSqError];
                
                % check if the error has increased
                if it > 1
                    if obj.avgWhiteSqErrorArray(end) >= obj.avgWhiteSqErrorArray(end-1)
                        % The avg error has increased. 
                        disp('error has increased!')
                        %break;
                        lambda = lambda * v
                    else
                        lambda = lambda / v
                    end
                end
                
                % LevenbergMarquardt
                %opts.POSDEF = true;
                opts.SYM = true;
                dx = linsolve(obj.A + lambda*diag(diag(obj.A)), obj.b, opts);
                
                % append deltas
                obj.deltaArray = [obj.deltaArray, dx];
                
                % apply the update.
                [graph] = obj.applyUpdate(graph, dx);
                
                %drawFrameGraph(graph);
                %drawnow;
                
                %graph.FrameContainer{1}.imustate.p
                %graph.FrameContainer{1}.imustate.R
                %graph.FrameContainer{1}.imustate.v
                
                if it >= niter
                    break;
                end
                
                % after update, recompute the residuals
                obj.computeResiduals(graph);
                
            end
            
        end
        
        function [] = computeResiduals(obj, graph)
            
            for idx = (1:length(obj.errorTermContainer))
                try
                    [r, information, J] = obj.errorTermContainer{idx}.computeResidual(graph);
                catch
                    disp('could not compute the residual');
                    s = struct('residual', [], 'information', [], 'jacobians', JacobianContainer());
                    obj.constraintBuffer{idx} = s;
                    continue;
                end
                
                s = struct('residual', r, 'information', information, 'jacobians', J);
                
                obj.constraintBuffer{idx} = s;
            end
            
        end
       
        
        % creates a sparse matrix, J, such that J*dx ~ r
        function [J] = createConstraintJacobian(obj, jacobianContainer, residualDim)
            if residualDim == 0
                error('constraint cannot have a 0 dimension residual!');
            end
            
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
        function [] = initialize(obj, graph)
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
                    pid = j{1};
                    lid = j{2};
                    %set this from the landmark itself
                    pinv = 1/graph.getLandmark(pid, lid).dinvPriorUncertainty;
                    if ~isfinite(pinv)
                        error('landmark information not finite!');
                    end
                    
                    obj.prior.addLandmark(pid, lid, pinv); % this checks if the landmark has already been added, and adds it.
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
        
        
        % marginalizes an imustate out of the problem by approximating all
        % error terms associated to the marginalized state with a single
        % quadratic error term.
        function [] = marginalizeImustate(obj, id)
            % move this variable set to the top of the problem.
            % VERY IMPORTANT: the prior must also be shifted with the
            % variables.
            key = obj.prior.indexHandler.imustateKey(id);
            [obj.prior.A, obj.prior.b] = obj.prior.indexHandler.moveVariableToTop(key, obj.prior.A, obj.prior.b);
            
            
            Am = zeros(obj.prior.indexHandler.dimensions());
            bm = zeros(obj.prior.indexHandler.dimensions(), 1);
            for c = obj.constraintBuffer
                for jc = c{1}.jacobians.imustateJacobians
                    if jc{1}{1} == id
                        try
                            J = obj.createConstraintJacobian(c{1}.jacobians, length(c{1}.residual));
                        catch
                            continue;
                        end
                        
                        Am = Am + J'*c{1}.information*J;
                        bm = bm - J'*c{1}.information*c{1}.residual;
                    end
                end
            end
            
            Am = Am + obj.prior.A;
            bm = bm + obj.prior.b;
            
            % use the schur complement to compute the conditional variance.
            mInd = obj.prior.indexHandler.getImustateIndices(id);
            rInd = ((mInd(end)+1):obj.prior.indexHandler.dimensions());
            
            bp = bm(rInd, 1) - Am(rInd, mInd) * inv(Am(mInd, mInd)) * bm(mInd, 1);
            Ap = Am(rInd, rInd) - Am(rInd, mInd) * inv(Am(mInd, mInd)) * Am(mInd, rInd);
            
            % remove the variables
            obj.prior.indexHandler.removeVariable(key);
            obj.prior.A = Ap;
            obj.prior.b = bp;
            %obj.prior.dx0 = zeros(obj.prior.indexHandler.dimensions(), 1);
            obj.prior.indexHandler.checkVariables();
        end
        
        
        % This function will marginalize a batch of landmarks
        % simultaneosly. The landmarkIDArray is structured as follows:
        % {{parentID, landmarkID}, {parentID, landmarkID}, ...}
        % The function will fail if the landmark is not in the variable
        % list of the optimizer right now.
        function [] = marginalizeLandmarkBatch(obj, landmarkIDArray)
            for pair = landmarkIDArray
                pid = pair{1}{1};
                lid = pair{1}{2};
                obj.marginalizeLandmark(pid, lid);
            end
        end
        
        
        % marginalizes a landmark out of the problem by approximating all
        % error terms associated to the marginalized state with a single
        % quadratic error term.
        function [] = marginalizeLandmark(obj, parentFrameID, landmarkID)
            % move this variable set to the top of the problem.
            % VERY IMPORTANT: the prior must also be shifted with the
            % variables.
            key = obj.prior.indexHandler.landmarkKey(parentFrameID, landmarkID);
            
            if obj.prior.indexHandler.hasKey(key)
                [obj.prior.A, obj.prior.b] = obj.prior.indexHandler.moveVariableToTop(key, obj.prior.A, obj.prior.b);
            else
                fprintf('Tried to marginalize a landmark not in the variables\n');
                return;
            end
            
            
            Am = zeros(obj.prior.indexHandler.dimensions());
            bm = zeros(obj.prior.indexHandler.dimensions(), 1);
            for c = obj.constraintBuffer
                for jc = c{1}.jacobians.landmarkJacobians
                    if jc{1}{1} == parentFrameID && jc{1}{2} == landmarkID
                        try
                            J = obj.createConstraintJacobian(c{1}.jacobians, length(c{1}.residual));
                        catch
                            continue;
                        end
                        
                        Am = Am + J'*c{1}.information*J;
                        bm = bm - J'*c{1}.information*c{1}.residual;
                    end
                end
            end
            
            Am = Am + obj.prior.A;
            bm = bm + obj.prior.b;
            
            % use the schur complement to compute the conditional variance.
            mInd = obj.prior.indexHandler.getLandmarkIndices(parentFrameID, landmarkID);
            rInd = ((mInd(end)+1):obj.prior.indexHandler.dimensions());
            
            bp = bm(rInd, 1) - Am(rInd, mInd) * inv(Am(mInd, mInd)) * bm(mInd, 1);
            Ap = Am(rInd, rInd) - Am(rInd, mInd) * inv(Am(mInd, mInd)) * Am(mInd, rInd);
            
            % remove the variables
            obj.prior.indexHandler.removeVariable(key);
            obj.prior.A = Ap;
            obj.prior.b = bp;
            %obj.prior.dx0 = zeros(obj.prior.indexHandler.dimensions(), 1);
            obj.prior.indexHandler.checkVariables();
        end
    end
end


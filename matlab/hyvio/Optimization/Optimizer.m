classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties (Access = private)
        constraintBuffer = {}; % stores all of the jacobians and residuals for the update. {residual, information, JacobianContainer}
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
        
        
    end
end


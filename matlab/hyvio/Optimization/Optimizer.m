classdef Optimizer < handle
    %OPTIMIZER The goal of this class is to handle the actual optimization
    %of the graph given a set of error terms.
    
    properties (Access = private)
        residualJacobianBuffer = {}; % stores all of the jacobians and residuals for the update. {residual, JacobianContainer}
        errorTermContainer = {}; % stores all error terms for this optimizer.
        
        id2IndexMap = containers.Map(); % use: map('m->n') => indices of landmark n from frame m in the update vector. n <= 0 for a frame's imustate reference
        maxIndex = 0; % this is used to keep track of the size of the update vector.
    end
    
    methods
        function [obj] = Optimizer()
            obj.id2IndexMap = containers.Map();
            obj.residualJacobianBuffer = {};
            obj.errorTermContainer = {};
        end
        
        function [] = addErrorTerm(obj, errorTerm)
            % adds a generic error term to the
            obj.errorTermContainer{end+1} = errorTerm;
        end
        
        
    end
end


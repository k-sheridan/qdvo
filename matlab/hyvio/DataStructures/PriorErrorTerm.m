classdef PriorErrorTerm < handle
    %PRIORERRORTERM stores the approximmated marginalized information in a
    %quadratic error term. It can be though of as a prior.
    
    properties
        
    end
    
    methods
        function obj = PriorErrorTerm(inputArg1,inputArg2)
            %PRIORERRORTERM Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end


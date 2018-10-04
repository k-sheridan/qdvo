classdef Patch
    %PATCH this class is used to compute the transformed patch from the
    %original frame. It also precomputes and stores some info.
    
    properties
        Property1
    end
    
    methods
        function obj = Patch(inputArg1,inputArg2)
            %PATCH Construct an instance of this class
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


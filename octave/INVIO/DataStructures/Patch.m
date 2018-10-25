classdef Patch
    %PATCH this class is used to compute the transformed patch from the
    %original frame. It also precomputes and stores some info.
    
    properties
        image
        meanIntensity
        sumZeroMeanSquared
    end
    
    methods
        function obj = Patch(patchImageData)
            %PATCH Construct an instance of this class
            obj.image = patchImageData;
            
            % compute the mean
            total = sum(sum(obj.image));
            
            obj.meanIntensity = total / numel(obj.image);
            
            % compute the (kind of) variance of the patch
            obj.sumZeroMeanSquared = sum(sum((obj.image - obj.meanIntensity).^2));
        end
        
    end
end


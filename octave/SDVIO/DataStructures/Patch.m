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
            sum = 0;
            for intensity = obj.image(:)'
                sum = sum + intensity
            end
            
            obj.meanIntensity = sum / numel(obj.image);
            
            % compute the (kind of) variance of the patch
            obj.sumZeroMeanSquared = 0;
            
            for intensity = obj.image(:)'
                obj.sumZeroMeanSquared = obj.sumZeroMeanSquared + (intensity - obj.meanIntensity)^2;
            end
            
        end
        
    end
end


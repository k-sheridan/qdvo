classdef IndexHandler < handle
    %INDEXHANDLER This class handles the mapping between a graph and linear system. 
    
    properties %(Access = private)
        id2IndexMap = containers.Map();
        maxIndex = 0;
    end
    
    methods
        function [key] = imustateKey(obj, frameID)
            key = sprintf('%i->0', frameID);
        end
        
        function [key] = landmarkKey(obj, parentFrameID, landmarkID)
            key = sprintf('%i->%i', parentFrameID, landmarkID);
        end
        
        function [] = reset(obj)
            obj.id2IndexMap = containers.Map();
            obj.maxIndex = 0;
        end
        
        % adds indices for a landmark
        function [] = addLandmark(obj, parentFrameID, landmarkID)
            dim = 1;
            key = obj.landmarkKey(parentFrameID, landmarkID);
            if ~obj.id2IndexMap.isKey(key)
                obj.id2IndexMap(key) = (obj.maxIndex+1:obj.maxIndex+dim);
                obj.maxIndex = obj.maxIndex+dim;
            end
        end
        
        % adds indices for a imustate
        function [] = addImustate(obj, frameID)
            dim = 15;
            key = obj.imustateKey(frameID);
            if ~obj.id2IndexMap.isKey(key)
                obj.id2IndexMap(key) = (obj.maxIndex+1:obj.maxIndex+dim);
                obj.maxIndex = obj.maxIndex+dim;
            end
        end
        
        % gets the indices of a landmark
        function [indices] = getLandmarkIndices(obj, parentFrameID, landmarkID)
            indices = obj.id2IndexMap(obj.landmarkKey(parentFrameID, landmarkID));
        end
        
        function [indices] = getImustateIndices(obj, frameID)
            indices = obj.id2IndexMap(obj.imustateKey(frameID));
        end
        
        % removes variable by moving it to the top of the system and
        % deleting it.
        function [A, b] = removeVariable(obj, key, A, b)
            
            if nargin == 2
                obj.moveVariableToTop(key);
            elseif nargin == 3
                [A] = obj.moveVariableToTop(key, A);
            elseif nargin == 4
                [A, b] = obj.moveVariableToTop(key, A, b);
            end
            
            removeIndices = obj.id2IndexMap(key);
            
            if nargin >= 2
                obj.id2IndexMap.remove(key);
                for k = obj.id2IndexMap.keys
                    obj.id2IndexMap(k{1}) = obj.id2IndexMap(k{1}) - removeIndices(end);
                end
            end
            
            if nargin >= 3
                A = A(removeIndices(end)+1:end, removeIndices(end)+1:end);
            end
            
            if nargin == 4
                b = b(removeIndices(end)+1:end, 1);
            end
        end
        
        % reorders the linear system such that the indices associated to
        % the key are at the top of the vector. The matrix and vector are optional.
        % Tested and functioning properly. should be used in try catch
        % statement.
        function [A, b] = moveVariableToTop(obj, key, Ainitial, binitial)
            % exploit the fact that all indices are unique.
            originalIndices = obj.id2IndexMap(key);
            shift = length(originalIndices); % this is the shift necessary to apply to all previous indices.
            
            for checkKey = obj.id2IndexMap.keys
                checkIndices = obj.id2IndexMap(checkKey{1});
                if checkIndices(1) < originalIndices(1)
                    obj.id2IndexMap(checkKey{1}) = checkIndices + shift;
                end
            end
            
            obj.id2IndexMap(key) = (1:length(originalIndices));
            
            % Linear system shift
            
            % shift A
            if nargin >= 3
                At = Ainitial;
                % first do a column shift.
                At(:, (1:length(originalIndices))) = Ainitial(:, originalIndices);
                At(:, (length(originalIndices)+1:originalIndices(end))) = Ainitial(:, (1:originalIndices(1)-1));
                
                % then do a row shift.
                A = At;
                A((1:length(originalIndices)), :) = At(originalIndices, :);
                A((length(originalIndices)+1:originalIndices(end)), :) = At((1:originalIndices(1)-1), :);
                
                
            end
            
            % shift b
            if nargin == 4
                b = binitial;
                
                b(1:length(originalIndices), :) = binitial(originalIndices, 1);
                b((length(originalIndices)+1:originalIndices(end)), 1) = binitial((1:originalIndices(1)-1), :);
            end
            
        end
    end
end


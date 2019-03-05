classdef IndexHandler < handle
    %INDEXHANDLER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Access = private)
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
        
        % adds indices on the map for all variables in the container
        function [] = addLandmark(obj, parentFrameID, landmarkID)
            dim = 1;
            key = obj.landmarkKey(parentFrameID, landmarkID);
            if ~obj.id2IndexMap.isKey(key)
                obj.id2IndexMap(key) = (obj.maxIndex+1:obj.maxIndex+dim);
                obj.maxIndex = obj.maxIndex+dim;
            end
        end
    end
end


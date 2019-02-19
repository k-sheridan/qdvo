classdef Frame < handle
    %FRAME Helper class for dealing with image
    
    properties
        raw_image % Raw pixel data (grayscale).
        maxIntensity % the maximum value the image can have
        t % Capture time in seconds
        cameraModel % an instance of the camera model. 
        imustate %IMUSTATE Stores the state of this keyframe.
        
        camID = 1;
        
        ID = -1 %INT unique keyframe id. This id is used to associate landmarks and measurements with keyframe.
        
        isKeyframe = false; % this flag classifies this frame as a keyframe.
        
        landmarks = {}; %In this method landmarks are represented in the frame from which they were first observed (empty if not keyframe).
        
    end
    
    methods
        function obj = Frame(image, maxIntensity, t, cameraModel, imuState)
            % constructs a frame object.
            obj.camID = 1;
            obj.raw_image = image;
            obj.t = t;
            obj.cameraModel = cameraModel;
            obj.maxIntensity = maxIntensity;
            
            if (nargin > 4)
                obj.imustate = imuState;
            else
                obj.imustate = IMUState();
            end
        end
        
        function [landmarkIndex] = getLandmarkIndex(obj, landmarkID)
            landmarkIndex = landmarkID - obj.landmarks{1}.ID + 1;
            
            % is this correct?
            if obj.landmarks{landmarkIndex}.ID ~= landmarkID
                disp('performing linear search to find frame index');
                landmarkIndex = -1; % this tells us if search failed
                for idx = (1:length(obj.landmarks))
                    if obj.landmarks{idx}.ID == landmarkID
                        landmarkIndex = idx;
                        break;
                    end
                end
            end
        end
    end
end


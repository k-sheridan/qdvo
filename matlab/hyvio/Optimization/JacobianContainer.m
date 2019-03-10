classdef JacobianContainer
    %JACOBIANCONTAINER stores jacobians and a reference to their variable ID
    
    properties
        % each of these store jacobians 
        landmarkJacobians = {}; %{{parentFrameID, LandmarkID, J}}
        imustateJacobians = {}; %{{obsFrameID, J}}
        extrinsicJacobians = {}; %{{key, J}}
        
    end 
    
    methods
        
    end
end


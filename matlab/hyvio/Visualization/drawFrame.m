function [] = drawFrame(frame, graph)
%DRAWFRAME draws the frame and its correspondence models

drawErrorBars = false;
drawGMMMeans = true;
drawCorrespondencePriors = true;


I = frame.raw_image / frame.maxIntensity;

if drawGMMMeans
    
    res = 100;
    cmap = spring(res);
    
    pxArr = [];
    colorArr = [];
    % get the observations in this frame
    fo = graph.FrameObservationContainer{graph.getFrameObservationsIndex(frame.ID)};
    
    s = Settings();
    theta = s.minimumNormalizedMatchCorrelation;
    
    idx = 1;
    
    for lo = fo.landmarkObservations
        
        for pc = lo{1}.potentialCorrespondenceSet
            pxArr(idx, 1:4) = [pc{1}.pixel(1), pc{1}.pixel(2), 1, 1];
            
            row = max(min(round(((pc{1}.score)) * res + 1) , res), 1);
            c = cmap(row, 1:3);
            
            colorArr(idx, 1:3) = c;
            
            idx = idx + 1;
            
        end
        
        
    end
    
    I = insertShape(I, 'FilledRectangle', pxArr,'color',colorArr);
end

if drawCorrespondencePriors
    
    res = 1000;
    cmap = hot(res);
    
    zmax = 10;
    
    pxArr = [];
    colorArr = [];
    % get the observations in this frame
    fo = graph.FrameObservationContainer{graph.getFrameObservationsIndex(frame.ID)};
   
    idx = 1;
    
    %ASSUMING: camera 1
    T_i_c = graph.extrinsics.getImu2CameraTransform(frame.camID);
    
    for lo = fo.landmarkObservations
        
        obsIdx = graph.getFrameIndex(lo{1}.observationFrameID);
        try
            parentIdx = graph.getFrameIndex(lo{1}.landmarkParentFrameID);
        catch
            continue;
        end
        landmarkIdx = graph.FrameContainer{parentIdx}.getLandmarkIndex(lo{1}.landmarkID);
        
        T_w_pi = graph.FrameContainer{parentIdx}.imustate.poseTransform();
        T_w_oi = graph.FrameContainer{obsIdx}.imustate.poseTransform();
        bearing = graph.FrameContainer{parentIdx}.landmarks{landmarkIdx}.bearing;
        dinv = graph.FrameContainer{parentIdx}.landmarks{landmarkIdx}.dinv;
        
        
        % project the landmark into the camera frame.
        % T_oc_pc = inv(T_w_oi * T_i_c) * T_w_pi * T_i_c
        T_oc_pc = inv(T_w_oi * T_i_c) * T_w_pi * T_i_c;
        
        r_o = T_oc_pc(1:3, 1:3) * ([bearing; 1] / dinv) + T_oc_pc(1:3, 4);
        
        % project the observation frame landmark into pixel space.
        try
            [px] = graph.FrameContainer{obsIdx}.cameraModel.project(r_o);
        catch
            continue;
        end
        
        
        pxArr(idx, 1:3) = [px', 2];
        
        z = r_o(3);
    
        row = max(min(round((zmax - z) / zmax * res + 1) , res), 1);
        c = cmap(row, 1:3);
        
        if isempty(lo{1}.potentialCorrespondenceSet)
            colorArr(idx, 1:3) = [0.5, 0, 1]; % failed correspondence color
        else
            colorArr(idx, 1:3) = c;
        end
        
        idx = idx + 1;
        
    end
    
    I = insertShape(I, 'FilledCircle', pxArr,'color',colorArr, 'Opacity', 0.9);
    
end


imagesc(I, [0, frame.maxIntensity]);
axis off;

daspect('auto');

% plot gmm gaussian
if drawErrorBars
    hold on
    for lo = fo.landmarkObservations
        % Plot gaussian ellipsoid fitted on each gmm
        [cov, mean] = lo{1}.computeGMMCovariance();
        
        [V, D] = eig(cov);
        
        xl = [mean, mean + V(1:2, 1) * sqrt(D(1, 1))];
        yl = [mean, mean + V(1:2, 2) * sqrt(D(2, 2))];
        
        line(xl(1, 1:2), xl(2, 1:2), 'Color','red', 'LineWidth', 1.5, 'LineStyle', '-');
        line(yl(1, 1:2), yl(2, 1:2), 'Color','green', 'LineWidth', 1.5, 'LineStyle', '-');
        
    end 
end



%title(sprintf('Frame %i', frame.ID));

end


function [] = drawFrame(frame, graph)
%DRAWFRAME draws the frame and its correspondence models

drawErrorBars = false;
drawGMMMeans = true;
drawCorrespondencePriors = false;


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
            
            row = min(round(((pc{1}.score - theta) / (1 - theta)) * res + 1) , res);
            c = cmap(row, 1:3);
            
            colorArr(idx, 1:3) = c;
            
            idx = idx + 1;
            
        end
        
        
    end
    
    I = insertShape(I, 'FilledRectangle', pxArr,'color',colorArr);
end

if drawCorrespondencePriors
    
    res = 100;
    cmap = hot(res);
    
    pxArr = [];
    colorArr = [];
    % get the observations in this frame
    fo = graph.FrameObservationContainer{graph.getFrameObservationsIndex(frame.ID)};
   
    idx = 1;
    
    for lo = fo.landmarkObservations
        
        
        
        
        pxArr(idx, 1:4) = [pc{1}.pixel(1), pc{1}.pixel(2), 1, 1];
        
        row = min(round(((pc{1}.score - theta) / (1 - theta)) * res + 1) , res);
        c = cmap(row, 1:3);
        
        colorArr(idx, 1:3) = c;
        
        idx = idx + 1;
        
    end
    
    I = insertShape(I, 'FilledRectangle', pxArr,'color',colorArr);
    
end


imshow(I);


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



title(sprintf('Frame %i', frame.ID));

end


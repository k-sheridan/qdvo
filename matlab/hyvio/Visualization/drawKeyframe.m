function [] = drawKeyframe(frame)
%DRAWKEYFRAME draws the keyframe image with features visualized.

%plot pixels and bearings
res = 1000;
zmax = 10;



cmap = hot(res);

I = frame.raw_image / frame.maxIntensity;

pxArr = [];
colorArr = [];

for idx = (1:length(frame.landmarks))
    if frame.landmarks{idx}.status ~= LandmarkStatus.MARGINALIZED
        if frame.landmarks{idx}.status == LandmarkStatus.ACTIVE
            pxArr(idx, 1:3) = [frame.landmarks{idx}.px(1), frame.landmarks{idx}.px(2), 2];
        else
            pxArr(idx, 1:3) = [frame.landmarks{idx}.px(1), frame.landmarks{idx}.px(2), 1];
        end
        
        z = 1/frame.landmarks{idx}.dinv;
        
        
        row = max(min(round((zmax - z) / zmax * res + 1) , res), 1);
        c = cmap(row, 1:3);
        
        
        colorArr(idx, 1:3) = c;
    else
        pxArr(idx, 1:3) = [frame.landmarks{idx}.px(1), frame.landmarks{idx}.px(2), 1];
        colorArr(idx, 1:3) = [0.4, 0.4, 0.4];
    end
    
    
end

I = insertShape(I, 'FilledCircle', pxArr,'color',colorArr, 'Opacity', 1);

imagesc(I, [0, frame.maxIntensity]);
daspect('auto');

title(sprintf('Keyframe %i', frame.ID));

end


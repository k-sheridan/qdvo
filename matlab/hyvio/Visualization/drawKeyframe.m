function [] = drawKeyframe(frame)
%DRAWKEYFRAME draws the keyframe image with features visualized.

%plot pixels and bearings
res = 1000;
zmax = 10;

circlerad = 3;

cmap = hot(res);

I = frame.raw_image / frame.maxIntensity;

pxArr = [];
colorArr = [];

for idx = (1:length(frame.landmarks))
    pxArr(idx, 1:3) = [frame.landmarks{idx}.px(1), frame.landmarks{idx}.px(2), circlerad];
    
    z = 1/frame.landmarks{idx}.dinv;
    
    row = min(round((zmax - z) / zmax * res + 1) , res);
    c = cmap(row, 1:3);
    
    colorArr(idx, 1:3) = c;
    
end

I = insertShape(I, 'FilledCircle', pxArr,'color',colorArr, 'Opacity', 1);

imshow(I);

title(sprintf('Keyframe %i', frame.ID));

end


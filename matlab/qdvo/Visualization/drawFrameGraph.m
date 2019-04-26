function [] = drawFrameGraph(graph, n)
%DRAWFRAMEGRAPH draws the last n keyframes and current frame on one plot.
if nargin < 2
    n = 1;
end

hFig = figure(1);
%set(hFig, 'Position', [0 0 1000 500])

clf

dim = ceil(sqrt(n+2));

% draw keyframes
kfCount = 1;
for idx = (1:length(graph.FrameContainer))
    if graph.FrameContainer{idx}.isKeyframe
        subplot(dim, dim, kfCount);
        
        drawKeyframe(graph.FrameContainer{idx});
        
        kfCount = kfCount + 1
    end
    
    if kfCount > n
        break;
    end
end

% draw currentFrame
subplot(dim, dim, n+1);
drawFrame(graph.FrameContainer{end}, graph)

% draw the graph
subplot(dim, dim, n+2);
drawGraph(graph, length(graph.FrameContainer))

end


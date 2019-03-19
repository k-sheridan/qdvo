g = vio.graph;
figure();
drawKeyframe(g.FrameContainer{1});

v = VideoWriter('test.mp4', 'MPEG-4');
v.FrameRate = 10;
open(v);

f = figure();
set(f, 'Position', [0,0,1500,750])
idx = 1;
for f = g.FrameContainer
    clf;
    subplot(1, 2, 1)
    drawFrame(f{1}, g);
    subplot(1, 2, 2)
    drawGraph(vio.graph, idx);
    xlim([-10, 5])
    ylim([-2, 2])
    zlim([-2, 2])
    view(-210, 17)
    idx = idx + 1;
    drawnow;
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);
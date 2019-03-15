g = vio.graph;
figure();
drawKeyframe(g.FrameContainer{1});

figure()
for f = g.FrameContainer
    drawFrame(f{1}, g);
    drawnow;
end
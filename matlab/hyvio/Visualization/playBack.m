g = vio.graph;
for f = g.FrameContainer
    drawFrame(f{1}, g);
    drawnow;
end
g = vio.graph;
figure();
drawKeyframe(g.FrameContainer{1});

figure()
for f = g.FrameContainer
    subplot(1, 2, 1)
    drawFrame(f{1}, g);
    subplot(1, 2, 2)
    draw3DCamera(f{1}.imustate.poseTransform(), 0.5, 'r')
    drawnow;
end
function [] = drawGraph(graph)
% Draw the graph in 3D.
daspect([1,1,1])
hold on
grid on
for idx = (1:length(graph.FrameContainer))
    sz = 0.1;
    T_w_i = graph.FrameContainer{idx}.imustate.poseTransform();
    T_i_c = graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{idx}.camID);
    T_w_c = T_w_i * T_i_c;
    
    draw3DCamera(T_w_c, sz);
    
    for lidx = 1:length(graph.FrameContainer{idx}.landmarks)
        u0 = graph.FrameContainer{idx}.landmarks{lidx}.bearing;
        dinv = graph.FrameContainer{idx}.landmarks{lidx}.dinv;
        
        p_inWorld = T_w_c(1:3, 1:3) * [u0;1] * (1/dinv) + T_w_c(1:3, 4);
        plot3(p_inWorld(1), p_inWorld(2), p_inWorld(3), 'ro');
    end
end

hold off
end


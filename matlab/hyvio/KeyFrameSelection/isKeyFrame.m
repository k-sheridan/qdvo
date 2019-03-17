function [isKeyframe] = isKeyframe(frameInQuestion, graph)
%ISKEYFRAME Checks if a frame is a keyframe given the graph.
% If the frame in question's distance from the last keyframe is far
% enough away in proportion to the average active feature distances or
% there are not enough observed features from the last keyframe in the
% current frame. It is called a keyframe.

isKeyframe = false;

kfIdx = -1;
s = Settings();

if graph.FrameObservationContainer{end} < s.minimumFeatures
    disp('creating keyframe due to low features')
    isKeyframe = true;
    return;
end

% find the last keyframe in the graph.
for idx = (length(graph.FrameContainer):-1:1)
    if graph.FrameContainer{idx}.isKeyframe
        kfIdx = idx;
        
        if kfIdx < 1
            isKeyframe = true;
            return
        else
            T_i_c = graph.extrinsics.getImu2CameraTransform(frameInQuestion.camID);
            T = inv(frameInQuestion.imustate.poseTransform() * T_i_c) * graph.FrameContainer{kfIdx}.imustate.poseTransform() * T_i_c
            
            transNorm = norm(T(1:3, 4));
            
            if length(graph.FrameContainer{kfIdx}.landmarks) < 1
                disp('no landmarks in keyframe!');
                continue;
            end
            
            avgSceneDepth = 0;
            for l = graph.FrameContainer{kfIdx}.landmarks;
                avgSceneDepth = avgSceneDepth + 1/l{1}.dinv;
            end
            
            avgSceneDepth = avgSceneDepth / length(graph.FrameContainer{kfIdx}.landmarks);
            
            ratio = transNorm / avgSceneDepth;
            
            if ratio >= s.trans2DepthRatio
                disp('creating keyframe due to high translation')
                isKeyframe = true;
                return
            end
            
        end
        
    end
end




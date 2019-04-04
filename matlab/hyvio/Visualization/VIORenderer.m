classdef VIORenderer < handle
    %VIORENDERER This will produce a pretty video of the VIO algorithm
    %running
    
    properties
        posHist = []; % [pos, pos, pos, ...]
        kfPosHist = []; % [pos, pos, pos, ...]
        marginalizedPoints = []; %MX3
        previousActiveKFID = []; % the active kfs at last update
        vw = VideoWriter('render.avi');
        
        
        kPos = 0.01;
        kTarget = 0.1;
        d = 5;
        cameraTarget = [0;0;0];
        cameraPos = [-1;0;0];
    end
    
    methods
        function obj = VIORenderer()
            
        end
        
        function [] = update(obj, graph)
            currentActiveKFID = [];
            setttings = Settings();
            % draw Keyframes
            AR = 16/9;
            height = 720;
            set(gcf, 'Position', [0,0,AR*height, height])
            set(gcf, 'Color', [0.4,0.4,0.4]);
            clf;
            
            
            obj.posHist = [obj.posHist, graph.FrameContainer{end}.imustate.p];
            if graph.FrameContainer{end}.isKeyframe
                obj.kfPosHist = [obj.kfPosHist, graph.FrameContainer{end}.imustate.p];
            end
            
            for idx = (length(graph.FrameContainer):-1:1)
                
                if graph.FrameContainer{idx}.isKeyframe && graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
                    currentActiveKFID = [currentActiveKFID, graph.FrameContainer{idx}.ID];
                end
            end
            
            
            % draw 3D stuff
            T_i_c = graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{end}.camID);
            subplot('Position', [0, 0.3, 0.65, 0.7]);
            
            hold on;
            plot3(obj.posHist(1, :), obj.posHist(2, :), obj.posHist(3, :), 'b-');
            plot3(obj.kfPosHist(1, :), obj.kfPosHist(2, :), obj.kfPosHist(3, :), 'w-');
            
            lookatArr = [];
            
            % draw current camera
            T_w_cc = graph.FrameContainer{end}.imustate.poseTransform() * T_i_c;
            s = draw3DCamera(T_w_cc, 0.3, 'g');
            lookatArr = [lookatArr, s];
            
            % draw keyframes
            for kfid = currentActiveKFID
                fidx = graph.getFrameIndex(kfid);
                
                if fidx ~= length(graph.FrameContainer)
                    T_w_kfc = graph.FrameContainer{fidx}.imustate.poseTransform() * T_i_c;
                    s = draw3DCamera(T_w_kfc, 0.2, 'w');
                    lookatArr = [lookatArr, s];
                end
                
            end
            
            
            
            for kfid = currentActiveKFID
                [pts, colors] = obj.createKFPointCloud(graph, kfid);
                if length(colors)
                    scatter3(pts(:, 1), pts(:, 2), pts(:, 3), 10, [colors, colors, colors], 'o', 'filled')
                end
            end
            
            
            % Run follow cam controller
            angle = pi/18;
            distance = obj.d;
            camPosSetPoint = T_w_cc(1:3, 1:3) * [0; -sin(angle); -cos(angle)]*distance + T_w_cc(1:3, 4);
            % do feedback update to the target and pos
            obj.cameraTarget = obj.cameraTarget + (T_w_cc(1:3, 4)-obj.cameraTarget)*obj.kTarget;
            obj.cameraPos = obj.cameraPos + (camPosSetPoint-obj.cameraPos)*obj.kPos;
            
            
            campos(obj.cameraPos);
            camtarget(obj.cameraTarget);
            ax = gca;
            ax.CameraViewAngle = 90;
            axis off;
            daspect([1,1,1])
            
            
            % draw 2d
            shift = 0;
            for idx = (length(graph.FrameContainer):-1:1)
                if idx == length(graph.FrameContainer)
                    subplot('Position', [0.65, 0.3, 0.35, 0.7]);
                    drawFrame(graph.FrameContainer{idx}, graph);
                end
                
                if graph.FrameContainer{idx}.isKeyframe && graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
                    
                    left = (shift) * 1/(setttings.windowSize-1);
                    subplot('Position', [left, 0, 1/(setttings.windowSize-1), 0.3]);
                    drawKeyframe(graph.FrameContainer{idx});
                    shift = shift + 1;
                end
            end
            
            
            %render
            set(gcf, 'Color', [0.4,0.4,0.4]);
            % render and save image
            set(gcf, 'Position', [0,0,AR*height, height])
            drawnow;
            
            frame = getframe(gcf);
            writeVideo(obj.vw,frame);
            
        end
        
        function [points, intensity] = createKFPointCloud(obj, graph, kfid)
            fidx = graph.getFrameIndex(kfid);
            
            T_i_c = graph.extrinsics.getImu2CameraTransform(graph.FrameContainer{fidx}.camID);
            T_w_c = graph.FrameContainer{fidx}.imustate.poseTransform() * T_i_c;
            
            points = [];
            intensity = [];
            
            for l = graph.FrameContainer{fidx}.landmarks
                if l{1}.status == LandmarkStatus.ACTIVE || l{1}.epipolarDepthEstimator.initialized
                    pt = T_w_c(1:3, 1:3) * [l{1}.bearing; 1] / l{1}.dinv + T_w_c(1:3, 4);
                    b = graph.FrameContainer{fidx}.raw_image(l{1}.px(2), l{1}.px(1));
                    
                    points = [points; pt'];
                    intensity = [intensity; b/graph.FrameContainer{fidx}.maxIntensity];
                end
            end
        end
        
    end
end


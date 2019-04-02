classdef VIORenderer < handle
    %VIORENDERER This will produce a pretty video of the VIO algorithm
    %running
    
    properties
        posHist = []; % [pos, pos, pos, ...]
        marginalizedPoints = []; %MX3
        previousActiveKFID = []; % the active kfs at last update
        vw = VideoWriter('render.avi');
        
        
        k = 5;
        cameraTarget = [0;0;0];
        cameraPos = [-5;0;0];
    end
    
    methods
        function obj = VIORenderer()
            
        end
        
        function [] = update(obj, graph)
            currentActiveKFID = [];
            
            s = Settings();
            % draw Keyframes
            AR = 16/9;
            height = 720;
            set(gcf, 'Position', [0,0,AR*height, height])
            set(gcf, 'Color', [0.4,0.4,0.4]);
            for idx = (length(graph.FrameContainer):-1:1)
                if idx == length(graph.FrameContainer)
                    subplot('Position', [0.65, 0.3, 0.35, 0.7]);
                    drawFrame(graph.FrameContainer{idx}, graph);
                    obj.posHist = [obj.posHist, graph.FrameContainer{idx}.imustate.p];
                end
                
                if graph.FrameContainer{idx}.isKeyframe && graph.FrameContainer{idx}.status == FrameStatus.ACTIVE
                    
                    currentActiveKFID = [currentActiveKFID, graph.FrameContainer{idx}.ID];
                    
                    left = (length(currentActiveKFID)-1) * 1/s.windowSize;
                    subplot('Position', [left, 0, 1/s.windowSize, 0.295]);
                    drawKeyframe(graph.FrameContainer{idx});
                end
            end
            
            % draw Point cloud
            subplot('Position', [0, 0.3, 0.65, 0.7]);
            hold on;
            plot3(obj.posHist(1, :), obj.posHist(2, :), obj.posHist(3, :), 'b-');
            
            axis off;
            
            % render and save image
            drawnow;
            
        end
    end
end


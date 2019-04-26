classdef RadialSearchPattern
    %RADIALSEARCHPATTERN
    
    properties
        pattern; % {[deltas for radius 1], [deltas for radius 2]} [dx;dy]
        rMax;
    end
    
    methods
        function obj = RadialSearchPattern(rMax)
            obj.rMax = rMax;
            rMax = round(rMax);
            res = round(rMax*2*pi*4);
            thetas = linspace(0,2*pi,res);
            mask = zeros(rMax*2+1);
            %mask(rMax+1,rMax+1) = 1;
            
            for r = (1:rMax)
                obj.pattern{r} = [];
                for th = thetas
                    dx = round([cos(th);sin(th)]*r);
                    if ~mask(dx(2)+rMax+1, dx(1)+rMax+1)
                        obj.pattern{r} = [obj.pattern{r}, dx];
                        mask(dx(2)+rMax+1, dx(1)+rMax+1) = r;
                    end
                end
            end
            
        end
        
        function [] = draw(obj)
            mask = zeros(obj.rMax*2+1);
            for rad = (1:length(obj.pattern))
                for idx = (1:length(obj.pattern{rad}))
                    dx = obj.pattern{rad}(1:2, idx);
                    assert(~mask(dx(2)+obj.rMax+1, dx(1)+obj.rMax+1))
                    mask(dx(2)+obj.rMax+1, dx(1)+obj.rMax+1) = rad;
                end
            end
            
            imagesc(mask, [0,obj.rMax])
        end
        
    end
end


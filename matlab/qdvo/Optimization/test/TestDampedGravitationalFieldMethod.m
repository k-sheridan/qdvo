% This method uses a damped gravitational field to determine the
% indirect residual. It relies on an efficiently calculated steady state
% solution. 

xl = -10;
xu = 30;
yl = -10;
yu = 30;

potentialSolutions = [[0;0], [20;4], [20;1], [20;2], [20;3], [20;15], [0;5]];
error = [0.1, 1, 1, 0.01, 0.01, 0.1, 0.001];
mass = exp(-error);

pointArray = [];
vectorArray = [];
magArray = [];

% generate force field vectors
resolution = 4;

for x = (xl:(1/resolution):xu)
    for y = (yl:(1/resolution):yu)
        % create the force
        force = [0;0];
        forceMag = 0;
        for index = (1:length(potentialSolutions))
            tmp = (mass(index) / norm(potentialSolutions(1:2, index) - [x;y])) * ((potentialSolutions(1:2, index) - [x;y])/norm(potentialSolutions(1:2, index) - [x;y]));
            force = force + tmp;
            forceMag = forceMag + norm(tmp);
        end
        pointArray = [pointArray, [x;y]];
        vectorArray = [vectorArray, 1e12 * sqrt(norm(force)) * force / norm(force)];
        magArray = [magArray, forceMag];
    end
end



% numerical solve

x = [10;2];
dx = [0;0];

cd = 0.5;

solArray = [];

dt = 0.1;

for t = (0:dt:1000)
    
    solArray = [solArray, x];
    
    % create the force
    force = [0;0];
    for index = (1:length(potentialSolutions))
         force = force +  (mass(index) / (norm(potentialSolutions(1:2, index) - x)^2 + 1)) * ((potentialSolutions(1:2, index) - x)/(norm(potentialSolutions(1:2, index) - x) + 1));
    end
  
    % add damping
    force = force - cd * dx;
    
    % integrate
    x = x + dx * dt + 0.5 * force * dt^2;
    dx = dx + force * dt;
    
end


%scatter(potentialSolutions(1, :), potentialSolutions(2, :), 100 * mass, 'o')
xlim([xl, xu])
ylim([xl, xu])
hold on
length(magArray)
pcolor(reshape(pointArray(1, :),[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]), reshape(pointArray(2, :),[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]), reshape(magArray,[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]));
shading interp
quiver(pointArray(1, :), pointArray(2, :), vectorArray(1, :), vectorArray(2, :))
colorbar

plot(solArray(1, :), solArray(2, :))




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

% generate force field vectors
for x = (xl:1:xu)
    for y = (yl:1:yu)
        % create the force
        force = [0;0];
        for index = (1:length(potentialSolutions))
            force = force + (mass(index) / norm(potentialSolutions(1:2, index) - [x;y])^2) * ((potentialSolutions(1:2, index) - [x;y])/norm(potentialSolutions(1:2, index) - [x;y]));
        end
        pointArray = [pointArray, [x;y]];
        vectorArray = [vectorArray, 1e12 * sqrt(norm(force)) * force / norm(force)];
    end
end



% numerical solve

x = [8;2];
dx = [0;0];

cd = 1;

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


scatter(potentialSolutions(1, :), potentialSolutions(2, :), 100 * mass, 'o')
xlim([xl, xu])
ylim([xl, xu])
hold on
quiver(pointArray(1, :), pointArray(2, :), vectorArray(1, :), vectorArray(2, :))

plot(solArray(1, :), solArray(2, :))




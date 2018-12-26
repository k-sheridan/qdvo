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
magArray = [];

% generate force field vectors
resolution = 4;

for x = (xl:(1/resolution):xu)
    for y = (yl:(1/resolution):yu)
        % create the force
        forceMag = 0;
        for index = (1:length(potentialSolutions))
            tmp = (mass(index) / sqrt(norm(potentialSolutions(1:2, index) - [x;y])));
            forceMag = forceMag + norm(tmp);
        end
        pointArray = [pointArray, [x;y]];
        magArray = [magArray, forceMag];
    end
end


%scatter(potentialSolutions(1, :), potentialSolutions(2, :), 100 * mass, 'o')
xlim([xl, xu])
ylim([xl, xu])
hold on
length(magArray)
pcolor(reshape(pointArray(1, :),[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]), reshape(pointArray(2, :),[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]), reshape(magArray,[(resolution*(xu-xl) + 1), (resolution*(yu-yl) + 1)]));
shading interp
colorbar




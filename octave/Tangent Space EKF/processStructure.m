function [points_covs_out] = processStructure(points_covs_in)

points_covs_out = points_covs_in;

num_points = length(points_covs_in(1, :)) / 3;

add_variance = (1e-4)^2;

for index = 1:num_points
    points_covs_out(1:3, (3*index - 2):(3*index)) = points_covs_in(1:3, (3*index - 2):(3*index)) + diag([add_variance, add_variance, add_variance]);
end

end


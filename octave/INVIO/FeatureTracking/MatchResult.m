classdef MatchResult
    %MATCHRESULT A simple datastructure to store the information returned
    %by the patch matcher.
    
    properties
        error = MatchError.NONE % of type MatchError.
        pixel % the level 0 pixel match.
        covariance % the level 0 pixel match uncertainty.
    end
    
end


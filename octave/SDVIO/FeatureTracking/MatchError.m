classdef MatchError
    %MATCHERROR simple enum to describe the error of the patch matcher.
    
    enumeration
        NONE, MATCH_SCORE_BELOW_THRESHOLD, MATCH_NOT_UNIQUE
    end
end


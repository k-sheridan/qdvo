function [landmarkObservationArray, graph] = computeCorrespondenceModels(frame, graph)
    s = Settings();

    % This will only compute correspondence models for active and visible
    % landmarks.
    
    if strcmp(s.patchComparison, 'ZNCC')
        [landmarkObservationArray, graph] = computeCorrespondenceModelsZNCC(frame, graph);
    elseif strcmp(s.patchComparison, 'BRIEF')
        [landmarkObservationArray, graph] = computeCorrespondenceModelsBRIEF(frame, graph);
    else
        error('unrecognized patch comparison method');
    end

end


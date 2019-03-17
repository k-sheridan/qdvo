function [landmarkObservationArray] = computeCorrespondenceModels(frame, graph)
    s = Settings();

    if strcmp(s.patchComparison, 'ZNCC')
        landmarkObservationArray = computeCorrespondenceModelsZNCC(frame, graph);
    elseif strcmp(s.patchComparison, 'BRIEF')
        landmarkObservationArray = computeCorrespondenceModelsBRIEF(frame, graph);
    else
        error('unrecognized patch comparison method');
    end

end


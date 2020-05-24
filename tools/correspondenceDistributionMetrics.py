
def computeCorrespondenceDistributionMetrics(trackingData):
    metrics = {}
    metrics['initialized_correspondence_distributions'] = {}
    metrics['initialized_correspondence_distributions']['frames'] = {}
    metrics['num_correspondence_distributions'] = {}
    metrics['num_correspondence_distributions']['frames'] = {}

    for frameNumber in trackingData['frames']:
            newestKeyframe = list(trackingData['frames'][frameNumber]['graph']['keyframes'].keys())[0]
            keyframes = trackingData['frames'][frameNumber]['graph']['keyframes']
            
            for keyframeKey in trackingData['frames'][frameNumber]['graph']['keyframes']:
               if keyframes[keyframeKey]['imustate']['time_ns'] > keyframes[newestKeyframe]['imustate']['time_ns'] and keyframes[newestKeyframe]['initialized']:
                   newestKeyframe = keyframeKey

            if not keyframes[newestKeyframe]['initialized']:
                continue

            nInit = 0
            for cd in keyframes[newestKeyframe]['correspondence_distributions']:
                if keyframes[newestKeyframe]['correspondence_distributions'][cd]['initialized']:
                    nInit += 1

            metrics['initialized_correspondence_distributions']['frames'][frameNumber] = nInit 
            metrics['num_correspondence_distributions']['frames'][frameNumber] = len(keyframes[newestKeyframe]['correspondence_distributions']) 

    return metrics


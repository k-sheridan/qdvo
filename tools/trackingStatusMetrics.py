
def computeTrackingStatusMetrics(trackingData):
    lostTrackingEvents = 0
    framesWithLostTracking = 0
    framesWithTracking = 0
    framesInitializing = 0
    lastTrackingStatus = 0

    LOST_TRACKING_STATUS = 2
    TRACKING_STATUS = 1
    INITIALIZING_STATUS = 0

    result = {}
    result['status'] = {}
    result['status']['frames'] = {}

    for frameNumber in trackingData['frames']:
        trackingStatus = trackingData['frames'][frameNumber]['tracking_status']
        result['status']['frames'][frameNumber] = trackingStatus

        if trackingStatus == LOST_TRACKING_STATUS and lastTrackingStatus != LOST_TRACKING_STATUS:
            lostTrackingEvents = lostTrackingEvents + 1

        if trackingStatus == LOST_TRACKING_STATUS:
            framesWithLostTracking = framesWithLostTracking + 1

        if trackingStatus == TRACKING_STATUS:
            framesWithTracking = framesWithTracking + 1

        if trackingStatus == INITIALIZING_STATUS:
            framesInitializing = framesInitializing + 1

        lastTrackingStatus = trackingStatus
        

    result['lost_tracking_events'] = lostTrackingEvents
    result['frames_with_lost_tracking'] = framesWithLostTracking
    result['frames_with_tracking'] = framesWithTracking
    result['frames_initializing'] = framesInitializing
    return result


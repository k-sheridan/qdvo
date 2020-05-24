
def computeSWEMetrics(trackingData):
    result = {}
    result['iteration_counts'] = {}
    result['initial_error'] = {}
    result['final_error'] = {}
    result['number_of_landmarks_in_window'] = {}

    result['iteration_counts']['frames'] = {}
    result['initial_error']['frames'] = {}
    result['final_error']['frames'] = {}
    result['number_of_landmarks_in_window']['frames'] = {}

    # This lets us know when the SWE was ran again.
    lastFramesInWindow = []
    for frameNumber in trackingData['frames']:
        framesInWindow = trackingData['frames'][frameNumber]['sliding_window_estimator']['frame_keys_in_window']
        iterations = trackingData['frames'][frameNumber]['sliding_window_estimator']['solve_result']['iteration_sse']
        landmarksInWindow = trackingData['frames'][frameNumber]['sliding_window_estimator']['landmark_keys_in_window']
        # check if the SWE has been ran in this iteration.
        if framesInWindow != lastFramesInWindow:
            lastFramesInWindow = framesInWindow
            result['iteration_counts']['frames'][frameNumber] = len(iterations)
            result['initial_error']['frames'][frameNumber] = iterations[0]
            result['final_error']['frames'][frameNumber] = iterations[-1]
            result['number_of_landmarks_in_window']['frames'][frameNumber] = len(landmarksInWindow)
    
    return result

def computeFEVOMetrics(trackingData):
    result = {}
    result['iteration_counts'] = {}
    result['initial_error'] = {}
    result['final_error'] = {}
    result['iteration_counts']['frames'] = {}
    result['initial_error']['frames'] = {}
    result['final_error']['frames'] = {}

    for frameNumber in trackingData['frames']:
        iterations = trackingData['frames'][frameNumber]['front_end_visual_odometry']['solve_result']['iteration_sse']
        result['iteration_counts']['frames'][frameNumber] = len(iterations)
        result['initial_error']['frames'][frameNumber] = iterations[0]
        result['final_error']['frames'][frameNumber] = iterations[-1]

    return result

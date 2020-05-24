from percentiles import *

def computeSWEMetrics(trackingData):
    startErrors = []
    finalErrors = []
    nIterations = []
    # This lets us know when the SWE was ran again.
    lastFramesInWindow = []
    for frameNumber in trackingData['frames']:
        framesInWindow = trackingData['frames'][frameNumber]['sliding_window_estimator']['frame_keys_in_window']
        iterations = trackingData['frames'][frameNumber]['sliding_window_estimator']['solve_result']['iteration_sse']
        # check if the SWE has been ran in this iteration.
        if framesInWindow != lastFramesInWindow:
            lastFramesInWindow = framesInWindow
            nIterations.append(len(iterations))
            startErrors.append(iterations[0])
            finalErrors.append(iterations[-1])

    result = {}
    result['iteration_counts'] = computePercentiles(nIterations, [0, 50, 90, 100])
    result['initial_error'] = computePercentiles(startErrors, [0, 50, 90, 100])
    result['final_error'] = computePercentiles(finalErrors, [0, 50, 90, 100])
    return result

def computeFEVOMetrics(trackingData):
    startErrors = []
    finalErrors = []
    nIterations = []
    for frameNumber in trackingData['frames']:
        iterations = trackingData['frames'][frameNumber]['front_end_visual_odometry']['solve_result']['iteration_sse']
        nIterations.append(len(iterations))
        startErrors.append(iterations[0])
        finalErrors.append(iterations[-1])

    result = {}
    result['iteration_counts'] = computePercentiles(nIterations, [0, 50, 90, 100])
    result['initial_error'] = computePercentiles(startErrors, [0, 50, 90, 100])
    result['final_error'] = computePercentiles(finalErrors, [0, 50, 90, 100])
    return result

import json
import csv
import numpy as np

# Takes tracking data, and ground truth data and returns a metrics json.
def computeMetrics(trackingData, groundtruth):
    # Extract the imu pose estimates per frame.
    imuPoseEstimates = {}
    for frameNumber in trackingData['frames']:
        graph = trackingData['frames'][frameNumber]['graph']
        currentFrameKey = str(graph['current_frame_key']['index']) + "-" + str(graph['current_frame_key']['generation']) 
        print(currentFrameKey)

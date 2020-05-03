import json
import csv
import numpy as np
import sophus
from scipy.spatial.transform import Rotation as R
import math

def quaternionToSO3(w, x, y, z):
    r = R.from_quat([w, x, y, z])
    return sophus.SO3(np.array(r.as_matrix()))

def getInterpolatedGroundTruthPose(targetTimeNanoSeconds, groundtruth):
    # Attempt to find a ground truth pose.
    groundtruthReader = csv.reader(groundtruth)
    # skip the header
    next(groundtruthReader)

    # Find the lower bound pose
    lowerBoundFound = False
    lowerBoundTime = -1
    lowerBoundPos = np.array([0,0,0])
    lowerBoundSO3 = sophus.SO3
    for row in groundtruthReader:
        gtTime = int(row[0]) 
        timeDelta = targetTimeNanoSeconds - gtTime
        if timeDelta < 0:
            break
        lowerBoundFound = True
        lowerBoundTime = gtTime
        lowerBoundPos = np.array([float(row[1]), float(row[2]), float(row[3])])
        lowerBoundSO3 = quaternionToSO3(float(row[4]), float(row[5]), float(row[6]), float(row[7]))
        
    # Find the upper bound pose.
    for row in groundtruthReader:
        gtTime = int(row[0]) 
        timeDelta = targetTimeNanoSeconds - gtTime
        if timeDelta >= 0:
            break
        upperBoundPos = np.array([float(row[1]), float(row[2]), float(row[3])])
        upperBoundSO3 = quaternionToSO3(float(row[4]), float(row[5]), float(row[6]), float(row[7]))

        dt = gtTime - lowerBoundTime
        ratio = (targetTimeNanoSeconds - lowerBoundTime) / dt
        pos = lowerBoundPos + (upperBoundPos - lowerBoundPos) * ratio
        rot = lowerBoundSO3 * (sophus.SO3.exp((lowerBoundSO3.inverse() * upperBoundSO3).log() * ratio))
        #print(f"{lowerBoundPos}, {pos}, {upperBoundPos}")
        #print(f"{lowerBoundSO3}, {rot}, {upperBoundSO3}")
        return pos, rot
    raise RuntimeError('Failed to get interpolated ground truth pose.')



# Takes tracking data, and ground truth data and returns a metrics json.
def computeMetrics(trackingData, groundtruth):
    # Extract the imu pose estimates per frame.
    # Extract the ground truth trajectory.
    imuPoseEstimates = {}
    imuGroundTruth = {}
    for frameNumber in trackingData['frames']:
        graph = trackingData['frames'][frameNumber]['graph']
        currentFrameKey = str(graph['current_frame_key']['index']) + "-" + str(graph['current_frame_key']['generation']) 
        imustate = graph['keyframes'][currentFrameKey]['imustate']
        imuPoseEstimates[frameNumber] = {}
        imuPoseEstimates[frameNumber]['pos'] = np.array(imustate['pos']['data'])
        rot = imustate['attitude']
        imuPoseEstimates[frameNumber]['so3'] = quaternionToSO3(rot['w'], rot['x'], rot['y'], rot['z']) 
        time = int(imustate['time_ns']) 
        imuPoseEstimates[frameNumber]['time_ns'] = time

        # Get the interpolated ground truth pose.
        gtPos, gtSO3 = getInterpolatedGroundTruthPose(time, groundtruth)
        imuGroundTruth['time_ns'] = time
        imuGroundTruth['pos'] = gtPos
        imuGroundTruth['so3'] = gtSO3

import json
import csv
import numpy as np
import sophuspy as sophus
from scipy.spatial.transform import Rotation as R
import math
from sim3TrajectoryError import *
from estimatorMetrics import *
from trackingStatusMetrics import *
from correspondenceDistributionMetrics import *
from percentiles import *


def prettyDict(d, indent=0):
    for key, value in d.items():
        print('  ' * indent + str(key))
        if isinstance(value, dict):
            if 'frames' in value.keys():
                prettyDict(
                    computePercentiles(list(value['frames'].values()),
                                       [0, 50, 90, 100]), indent + 1)
            else:
                prettyDict(value, indent + 1)
        else:
            print('  ' * (indent + 1) + str(value))


# Takes tracking data, and ground truth data and returns a metrics json.
# output: Metrics dict
def computeMetrics(trackingData, groundtruth):
    print(f"Computing SIM3 Trajectory Error.")
    # Extract the imu pose estimate and GT.
    imuPoseEstimates, imuGroundTruth = extractImuTrajectories(
        trackingData, groundtruth)

    # Compute the trajectory error.
    posRMSE, rotRMSE, scaleDifference, oepf, tepf, bepf, sepf = computeTrajectoryError(
        imuGroundTruth, imuPoseEstimates)
    metrics = {}
    metrics['trajectory_rsme'] = {}
    metrics['trajectory_rsme']['position_m'] = posRMSE
    metrics['trajectory_rsme']['rotation_rad'] = rotRMSE
    metrics['scale_ratio'] = scaleDifference

    metrics['odometry_error'] = {}
    metrics['odometry_error']['rotation_error_deg'] = oepf
    metrics['odometry_error']['translation_error_m'] = tepf
    metrics['odometry_error']['bearing_error_deg'] = bepf
    metrics['odometry_error']['scale_error'] = sepf

    metrics['sliding_window_estimator_metrics'] = computeSWEMetrics(
        trackingData)
    metrics['frontend_visual_odometry_metrics'] = computeFEVOMetrics(
        trackingData)

    metrics['tracking_status_metrics'] = computeTrackingStatusMetrics(
        trackingData)

    metrics[
        'correspondence_distribution_metrics'] = computeCorrespondenceDistributionMetrics(
            trackingData)

    # Print the metrics.
    print("Metrics:")
    prettyDict(metrics, 1)

    return metrics

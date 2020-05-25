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
    lowerBoundSO3 = sophus.SO3()
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

# Given two trajectories, sim3 align the two and compute the SSE trajectory error and scale error.
# output: [translation error RMSE, rotation error RMSE, scale difference]
def computeTrajectoryError(trajectory1, trajectory2):
    # Compute the sim3 transform between the two trajectories.
    # dR = (R1.inverse() * R_sim * R2).log()
    # dt = (t1 - (R_sim * t2) * scale + t_sim)
    #
    # dt(phi) = (t1 - (R_sim * exp(phi) * t2) * scale + t_sim)
    # dt(phi) ~= (t1 - (R_sim * (I + hat(phi)) * t2) * scale + t_sim)
    # 
    # dt(phi)_dphi = R_sim * hat(t2 * scale) 
    #
    # dt(dt)_dt = 1
    #
    # dt(dscale)_dscale = -R_sim * t2 
    #
    #
    # dR(phi) = (R1.inverse() * R_sim * exp(phi) * R2).log()
    # dR(phi) = (R1.inverse() * R_sim * R2 * exp(R2.inverse() * phi)).log()
    # dR(phi) ~= (R1.inverse() * R_sim * (I + hat(phi)) * R2).log()
    #
    # dR(phi)_dphi = Jrinv((R1.inverse() * R_sim * R2).log()) * R2.inverse()

    # Order: [translation, rotation, scale]
    R_sim = sophus.SO3.exp([0,0,0])
    t_sim = np.zeros([3,1])
    scale = 1.0

    # dx order: [translation, rotation, scale]
    # output: [rotation, translation, scale]
    def applyUpdate(R_sim, t_sim, scale, dx):
        return R_sim * sophus.SO3.exp(dx[3:6, 0:1]), t_sim + dx[0:3, 0:1], scale + dx[6:7, 0:1]

    # output: [dt, dR]
    def computeError(R1, t1, R2, t2, R_sim, t_sim, scale):
        t1 = t1.reshape((3, 1))
        t2 = t2.reshape((3, 1))
        t_sim = t_sim.reshape((3, 1))

        dR = (R1.inverse() * R_sim * R2).log().reshape((3, 1))
        dt = (t1 - (R_sim * t2).reshape((3, 1)) * scale + t_sim).reshape((3, 1))
        return dt, dR

    # variable order: [translation, rotation, scale]
    # output R_6_7: [dt_dx, dR_dx]
    def computeJacobian(R1, t1, R2, t2, R_sim, t_sim, scale):
        t1 = t1.reshape((3, 1))
        t2 = t2.reshape((3, 1))
        t_sim = t_sim.reshape((3, 1))

        delta = 1e-4
        J = np.zeros([6, 7])
        for i in range(0, 7):
            dx = np.zeros([7, 1])
            dx[i:i+1, 0:1] = delta
            R_simHigh, t_simHigh, scaleHigh = applyUpdate(R_sim, t_sim, scale, dx)
            dt, dR = computeError(R1, t1, R2, t2, R_simHigh, t_simHigh, scaleHigh)
            errorHigh = np.zeros([6, 1])
            errorHigh[0:3, 0:1] = dt
            errorHigh[3:6, 0:1] = dR

            dx[i:i+1, 0:1] = -delta
            R_simLow, t_simLow, scaleLow = applyUpdate(R_sim, t_sim, scale, dx)
            dt, dR = computeError(R1, t1, R2, t2, R_simLow, t_simLow, scaleLow)
            errorLow = np.zeros([6, 1])
            errorLow[0:3, 0:1] = dt
            errorLow[3:6, 0:1] = dR

            J[0:6, i:i+1] = (errorHigh - errorLow) / (2 * delta) 

        return J

    def computeUpdate():
        H = np.zeros([7, 7]) 
        b = np.zeros([7, 1])
        for frameNumber in trajectory1:
            T1 = trajectory1[frameNumber]
            T2 = trajectory2[frameNumber]
            dt, dR = computeError(T1['so3'], T1['pos'], T2['so3'], T2['pos'], R_sim, t_sim, scale)
            e = np.zeros([6, 1])
            e[0:3, 0:1] = dt
            e[3:6, 0:1] = dR
            J = computeJacobian(T1['so3'], T1['pos'], T2['so3'], T2['pos'], R_sim, t_sim, scale)
            H = H + J.transpose().dot(J)
            b = b + -J.transpose().dot(e) 
        return np.linalg.inv(H).dot(b)

    # output: [SSE pos, SSE rot]
    def computeSSE():
        sqRotError = 0.0
        sqPosError = 0.0
        for frameNumber in trajectory1:
            T1 = trajectory1[frameNumber]
            T2 = trajectory2[frameNumber]
            dt, dR = computeError(T1['so3'], T1['pos'], T2['so3'], T2['pos'], R_sim, t_sim, scale)
            sqPosError += dt.transpose().dot(dt)  
            sqRotError += dR.transpose().dot(dR)  
        return sqPosError[0,0], sqRotError[0,0]

    def computeOdometryError():
        translationErrorPerFrame = {}
        translationBearingErrorPerFrame = {}
        orientationErrorPerFrame = {}
        translationErrorPerFrame['frames'] = {}
        translationBearingErrorPerFrame['frames'] = {}
        orientationErrorPerFrame['frames'] = {}
        scaleErrorPerFrame = {}
        scaleErrorPerFrame['frames'] = {}

        for frameNumber in trajectory1:
            try:
                T1 = trajectory1[frameNumber]
                T2 = trajectory2[frameNumber]
                T1_n = trajectory1[str(int(frameNumber)+1)]
                T2_n = trajectory2[str(int(frameNumber)+1)]
                dR1 = T1['so3'].inverse() * T1_n['so3']
                dR2 = T2['so3'].inverse() * T2_n['so3']
                dt1 = T1_n['pos'] - T1['pos']
                dt2 = (T2_n['pos'] - T2['pos']) * scale

                orientationErrorPerFrame['frames'][frameNumber] = 180 / math.pi * np.linalg.norm((dR1.inverse() * dR2).log())
                translationErrorPerFrame['frames'][frameNumber] = np.linalg.norm(dt2 - dt1)
                v1 = (dt1/np.linalg.norm(dt1))
                v2 = (dt2/np.linalg.norm(dt2))
                translationBearingErrorPerFrame['frames'][frameNumber] = abs(180/math.pi * math.acos(np.dot(np.squeeze(np.asarray(v2)), np.squeeze(np.asarray(v1))))) 
                scaleErrorPerFrame['frames'][frameNumber] = np.linalg.norm(dt1)/np.linalg.norm(dt2)
            except:
                continue
        return orientationErrorPerFrame, translationErrorPerFrame, translationBearingErrorPerFrame, scaleErrorPerFrame

    initialError = computeSSE()
    currentError = initialError
    for i in range(1, 20):
        update = computeUpdate()
        R_sim, t_sim, scale = applyUpdate(R_sim, t_sim, scale, update)
        error = computeSSE()
        if error[0] > currentError[0]:
            R_sim, t_sim, scale = applyUpdate(R_sim, t_sim, scale, -1 * update)
            #print(f"Ran {i+1} iterations")
            break
        else:
            currentError = error
    #print(f"SSE trajectory error {initialError} -> {currentError}")
    posRMSE = math.sqrt(currentError[0] / len(trajectory1))
    rotRMSE = math.sqrt(currentError[1] / len(trajectory1))

    rotErrorPerFrame, transErrorPerFrame, bearingErrorPerFrame, scaleErrorPerFrame = computeOdometryError()
    return posRMSE, rotRMSE, abs(scale[0,0]), rotErrorPerFrame, transErrorPerFrame, bearingErrorPerFrame, scaleErrorPerFrame

def extractImuTrajectories(trackingData, groundtruth):
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
        try:
            gtPos, gtSO3 = getInterpolatedGroundTruthPose(time, groundtruth)
        except:
            print(f"Failed to get interpolated ground truth pose for frame {frameNumber}");
            imuPoseEstimates.pop(frameNumber)
            continue
        imuGroundTruth[frameNumber] = {}
        imuGroundTruth[frameNumber]['time_ns'] = time
        imuGroundTruth[frameNumber]['pos'] = gtPos
        imuGroundTruth[frameNumber]['so3'] = gtSO3
        
    return imuPoseEstimates, imuGroundTruth


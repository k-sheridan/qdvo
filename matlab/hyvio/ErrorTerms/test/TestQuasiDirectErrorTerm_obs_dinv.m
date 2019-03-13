% randomly test the quasidirect error term.
cm = load('/Users/kevinsheridan/Documents/Mac Library/RnD/hyvio/matlab/hyvio/ErrorTerms/test/testCameraModel.mat');

cm = cm.cm;

for testNumber = (1:100)
    u0 = ([rand; rand]-0.5) * 4;
    dinv = rand * 10;
    
    l = Landmark();
    l.dinv = dinv;
    l.bearing = u0;
    l.ID = 1;
    l.frameID = 1;
    
    g = Graph();
    
    imu1 = IMUState();
    
    imu1.p = 0.1*[rand;rand;rand];
    imu1.R = so3Exp(0.05*[rand;rand;rand]);
    
    f1 = Frame([], 0, 0, cm, imu1);
    
    imu2 = IMUState();
    
    imu2.p = 0.1*[rand;rand;rand];
    imu2.R = so3Exp(0.05*[rand;rand;rand]);
    
    f2 = Frame([], 0, 0, cm, imu2);
    
    f1.ID = 1;
    f2.ID = 2;
    
    g.addFrame(f1);
    g.addFrame(f2);
    g.FrameContainer{1}.landmarks{1} = l;
    
    g.extrinsics.addIMU2CameraExtrinsic(1, rotx(90) * so3Exp(0.05*[rand;rand;rand]), 0.1*[rand;rand;rand]);
    
    lo = LandmarkObservation();
    
    % get the pixel position
    T = inv(f2.imustate.poseTransform() * g.extrinsics.getImu2CameraTransform(1)) * (f1.imustate.poseTransform() * g.extrinsics.getImu2CameraTransform(1));
    
    p_obs = T(1:3, 1:3) * [u0;1] * (1/dinv) + T(1:3, 4);
    
    px = cm.project(p_obs)
    
    pc = PotentialCorrespondence();
    pc.pixel = px;
    pc.score = 1;
    
    lo.theta = 0.9;
    lo.potentialCorrespondenceSet{1} = pc;
    lo.landmarkID = 1;
    lo.landmarkParentFrameID = 1;
    lo.observationFrameID = 2;
    
    et = QuasiDirectErrorTerm_obsFrame_landmarkDinv(lo);
    
    [res, pinv, jacs] = et.computeResidual(g)
    
end
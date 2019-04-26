% this checks that the inertial error term is correct.
cm = load('/Users/kevinsheridan/Documents/Mac Library/RnD/hyvio/matlab/hyvio/ErrorTerms/test/testCameraModel.mat');
cm = cm.cm;

g = Graph();

imu1 = IMUState();

imu1.p = zeros(3,1);
imu1.R = rotx(0);

f1 = Frame([], 0, 0, cm, imu1);

imu2 = IMUState();

imu2.p = zeros(3,1);
imu2.R = rotx(0);

f2 = Frame([], 0, 0.1, cm, imu2);

f1.ID = 1;
f2.ID = 2;

g.addFrame(f1);
g.addFrame(f2);

z1 = IMUMeasurement();
z2 = IMUMeasurement();

z1.gyro = [0.1;0;0];
z2.gyro = [0;0.2;0];
z1.accel = z2.gyro;
z2.accel = z1.gyro;
z1.accelRandomWalk = zeros(3,1);
z1.accelNoise = 1e-3*zeros(3,1);
z2.accelRandomWalk = zeros(3,1);
z2.accelNoise = 1e-3*zeros(3,1);
z1.gyroRandomWalk = zeros(3,1);
z1.gyroNoise = 1e-3*zeros(3,1);
z2.gyroRandomWalk = zeros(3,1);
z2.gyroNoise = 1e-3*zeros(3,1);
z1.t = 0.01;
z2.t = 0.06;

preintimu = PreintegratedIMUMeasurement();
preintimu.imuMeasurementArray = {z1,z2};
preintimu.childFrameID = f2.ID;
preintimu.parentFrameID = f1.ID;

dbias = zeros(6,1);
delta = 1e-9;

biasJacs = zeros(9, 6);
for idx = (1:6)
    dbias(idx) = dbias(idx) + delta;
    preintimu.preintegrateIMUMeasurements(dbias, f1.t, f2.t);
    dRHigh = preintimu.deltaR;
    dpHigh = preintimu.deltaP;
    dvHigh = preintimu.deltaV;
    
    dbias(idx) = dbias(idx) - 2*delta;
    preintimu.preintegrateIMUMeasurements(dbias, f1.t, f2.t);
    dRLow = preintimu.deltaR;
    dpLow = preintimu.deltaP;
    dvLow = preintimu.deltaV;
    
    biasJacs(1:9, idx) = [so3Log(dRLow'*dRHigh); dpHigh - dpLow; dvHigh - dvLow] / (2*delta);
end

biasJacs
clear
syms phix phiy phiz dtx dty dtz ddinv

assume([phix,phiy,phiz, dtx, dty, dtz], 'real')

A = so3Exp(1*[1,3,3]);
B = so3Exp(1*[1,4,3]);
C = so3Exp(1*[1,2,3]);
d = 1*[1;2;3];
e = 1*[1;5;3];
f = 1*[1;2;3];
dinv = 0.2;
u0 = [0;0;1];

theta = norm([phix,phiy,phiz]);
K = so3Hat([phix,phiy,phiz]/theta);
exp_dphi = eye(3) + sin(theta)*K + (1 - cos(theta))*K^2;
dt = [dtx;dty;dtz];

isHat = double(subs([diff(exp_dphi, phix), diff(exp_dphi, phiy), diff(exp_dphi, phiz)], [phix,phiy,phiz], 1e-24*[1,1,1]))

%proj = C'*(A*exp_dphi)'*B*C*u0*1/dinv + C'*(A*exp_dphi)'*(B*f + e - A*exp_dphi*f - (d+dt));
T = [C, f; zeros(1, 3), 1];
trans = inv([A*exp_dphi, d+dt; zeros(1, 3), 1] * T) * [B, e; zeros(1, 3), 1] * T;
proj = trans(1:3, 1:3) * u0/(dinv+ddinv) + trans(1:3, 4);
u = [proj(1)/proj(3); proj(2)/proj(3)];

p_obs = C'*(A)'*B*C*u0*1/dinv + C'*(A)'*(B*f + e - A*f - (d))
dPi = [1/p_obs(3), 0, -p_obs(1)/p_obs(3)^2;
    0, 1/p_obs(3), -p_obs(2)/p_obs(3)^2];

symJac = double(subs([diff(u, phix), diff(u, phiy), diff(u, phiz)], [phix,phiy,phiz, dtx,dty,dtz, ddinv], 1e-24*[1,1,1,1,1,1,1]))
%symJac = double([diff(u, phix), diff(u, phiy), diff(u, phiz)])
analJac = dPi * (C'*so3Hat(A'*(B*C*u0*(1/dinv) + B*f + e - d)))

symJac = double(subs([diff(u, dtx), diff(u, dty), diff(u, dtz)], [phix,phiy,phiz, dtx,dty,dtz,ddinv], 1e-24*[1,1,1,1,1,1,1]))
analJac = dPi * -(C'*A')

symJac = double(subs([diff(u, ddinv)], [phix,phiy,phiz, dtx,dty,dtz,ddinv], 1e-24*[1,1,1,1,1,1,1]))
analJac = dPi * -(C'*A'*B*C*u0*1/dinv^2)

% check that error term is functioning properly.
cm = load('/Users/kevinsheridan/Documents/Mac Library/RnD/hyvio/matlab/hyvio/ErrorTerms/test/testCameraModel.mat');
cm = cm.cm;

px0 = cm.project(p_obs)

l = Landmark();
l.dinv = dinv;
l.bearing = u0(1:2);
l.ID = 1;
l.frameID = 1;

g = Graph();

imu1 = IMUState();

imu1.p = e;
imu1.R = B;

f1 = Frame([], 0, 0, cm, imu1);

imu2 = IMUState();

imu2.p = d;
imu2.R = A;

f2 = Frame([], 0, 0, cm, imu2);

f1.ID = 1;
f2.ID = 2;

g.addFrame(f1);
g.addFrame(f2);
g.FrameContainer{1}.landmarks{1} = l;

g.extrinsics.addIMU2CameraExtrinsic(1, C, f);

lo = LandmarkObservation();

% get the pixel position
T = inv(f2.imustate.poseTransform() * g.extrinsics.getImu2CameraTransform(1)) * (f1.imustate.poseTransform() * g.extrinsics.getImu2CameraTransform(1));

p_obs = T(1:3, 1:3) * [u0(1:2);1] * (1/dinv) + T(1:3, 4);

%px = cm.project(p_obs) + [randn;randn];
[px, projac] = cm.project(p_obs)

resExpect = [0.1;-0.1];

pc = PotentialCorrespondence();
pc.pixel = px + resExpect;
pc.score = 1;

lo.theta = 0.9;
lo.potentialCorrespondenceSet{1} = pc;
lo.landmarkID = 1;
lo.landmarkParentFrameID = 1;
lo.observationFrameID = 2;

et = QuasiDirectErrorTerm_obsFrame_landmarkDinv(lo);

[res, pinv, jacs] = et.computeResidual(g)

etJac = inv(projac)*jacs.imustateJacobians{1}{2}
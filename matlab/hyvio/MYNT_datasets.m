% This is meant to test and evaluate my visual inertial slam method on all
% TUM VI datasets
clear all
%% DATASET
datasetPath = 'datasets/fountain/';

% CALIBRATION LEFT: width: 752, height: 480, fx: 364.61223237047539669, fy: 364.70254209576665971, cx: 385.60887400782206669, cy: 232.65626547906052224, model: 0, coeffs: [-0.31116718665000576, 0.08962547701688039, -0.00005212461326460, 0.00009844260968483, 0.00000000000000000

%vignette = imread('datasets/dataset-room1_512_16/dso/cam0/vignette.png', 'PNG'); % used as mask for feature tracking and selection


%% CALIBRATION
% Precalibrated extrinsic and intrinsics. These will be refined / estimated
% in the pipeline, but are used as good initial guesses.
focalLength = [190.97847715128717, 190.9733070521226];
principalPoint = [254.93170605935475, 256.8974428996504];
distortionCoefficients = [0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182];

accelBias = [-1.30318 -0.391441  0.380509]';
gyroBias = [0.0283122 0.00723077  0.0165292]';

gyroNoise = 0.00016;
accelNoise = 0.0028;

accelRandomWalk = 0.00086;
gyroRandomWalk = 2.2e-05;

accelScale = [1.00422, 0, 0;
                -7.82123e-05, 1.00136, 0;
                -0.0097745, -0.000976476, 0.970467];
gyroScale = [0.943611, 0.00148681, 0.000824366;
            0.000369694, 1.09413, -0.00273521;
             -0.00175252, 0.00834754, 1.01588];

T_camera0FromImu = [[-0.99954072 0.02910045 -0.00845616 0.04812531];
[ 0.00741901 -0.03556579 -0.9993398 -0.04626899];
[-0.02938199 -0.99894356 0.03533356 -0.06808129];
[ 0. 0. 0. 1. ]];

%% SETUP
% Create a camera model instance 
cameraModel = EquidistantCameraModel(distortionCoefficients, 1.44*2, focalLength, principalPoint, [512;512], 10000, vignette);

maxIntensity = 2^16;

% Create a settings struct
settings = Settings(); % a default settings file for the VIO impl to use
settings.initial_T_camFromImu = T_camera0FromImu;
settings.initial_accelBias = accelBias;
settings.initial_gyroBias = gyroBias;
settings.initial_accelScale = accelScale;
settings.initial_gyroScale = gyroScale;

% Create an instance of a VIO
vio = VIO(settings);

% create a renderer
renderer = VIORenderer();
open(renderer.vw);

%% RUN

% Load csv files for imu, camera
cam0 = readtable(sprintf('%scam0/data.csv', datasetPath));
imu0 = readtable(sprintf('%simu0/data.csv', datasetPath));

% these indices are the current 
imuIndex = 1;
camIndex = 1; % this can be set to specify the start point.

imuEnd = height(imu0);
camEnd = height(cam0); % this can be manually set to specify the end point

%camEnd = 700;

while (camIndex <= camEnd)
    if(cam0(camIndex, 1).x_timestamp_ns_ <=  imu0(imuIndex, 1).x_timestamp_ns_)
        % skip every other frame
%         if mod(camIndex, 2)
%             camIndex = camIndex + 1;
%             continue;
%         end
        
        % Add a frame to vio here
        rawImage = double(imread(sprintf('%scam0/data/%s', datasetPath, cam0(camIndex, 2).filename{1})));
        
        vio.addFrame(Frame(rawImage, maxIntensity, cam0(camIndex, 1).x_timestamp_ns_ * 1e-9, cameraModel)); % add the frame
        
        camIndex = camIndex + 1;
        
        % draw
        if vio.graph.FrameContainer{end}.isKeyframe || true
            renderer.update(vio.graph);
        end
        %drawFrameGraph(vio.graph);
        %drawnow
        
    else
        % Add the IMU measurement to vio here
        z = IMUMeasurement();
        
        temp = imu0(imuIndex, 2:7);
        z.gyro = [temp(1, 1).w_RS_S_x_radS__1_; temp(1, 2).w_RS_S_y_radS__1_; temp(1, 3).w_RS_S_z_radS__1_];
        z.accel = [temp(1, 4).a_RS_S_x_mS__2_; temp(1, 5).a_RS_S_y_mS__2_; temp(1, 6).a_RS_S_z_mS__2_];
        
        z.t = imu0(imuIndex, 1).x_timestamp_ns_ * 1e-9;
        
        z.accelRandomWalk = ones(3, 1)*accelRandomWalk;
        z.gyroRandomWalk = ones(3, 1)*gyroRandomWalk;
        
        z.accelNoise = ones(3, 1)*accelNoise;
        z.gyroNoise = ones(3, 1)*gyroNoise;
        
        vio.addIMUMeasurement(z); % add to vio
        
        imuIndex = imuIndex + 1;
    end
end

close(renderer.vw);


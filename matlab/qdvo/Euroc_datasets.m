% This is meant to test and evaluate my visual inertial slam method on all
% TUM VI datasets
clear all
%% DATASET
datasetPath = 'datasets/vicon_easy1/mav0/';

vignette = ones(480, 752); % used as mask for feature tracking and selection


%% CALIBRATION
% Precalibrated extrinsic and intrinsics. These will be refined / estimated
% in the pipeline, but are used as good initial guesses.
focalLength = [458.654, 457.296];
principalPoint = [367.215, 248.375];
distortionCoefficients = [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05];

accelBias = [0, 0, 0]';
gyroBias = [-0.0020943951023931952, 0.017453292519943295, 0.07749261878854824]';

gyroNoise = 0.00016;
accelNoise = 0.0028;

accelRandomWalk = 0.00086;
gyroRandomWalk = 2.2e-05;

accelScale = eye(3);
gyroScale = eye(3);

T_camera0FromImu = inv([0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975;
         0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768;
        -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949;
         0.0, 0.0, 0.0, 1.0]);

%% SETUP
% Create a camera model instance 
cameraModel = RadTanCameraModel(distortionCoefficients, pi/2, focalLength, principalPoint, [752; 480], 5000, vignette);

maxIntensity = 2^8;

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
camIndex = 90; % this can be set to specify the start point.

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
        
        %vio.addIMUMeasurement(z); % add to vio
        
        imuIndex = imuIndex + 1;
    end
end

close(renderer.vw);


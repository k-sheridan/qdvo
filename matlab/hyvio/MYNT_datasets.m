% This is meant to test and evaluate my visual inertial slam method on all
% TUM VI datasets
clear all
%% DATASET
datasetPath = 'datasets/fountain/';

vignette = ones(480, 752); % used as mask for feature tracking and selection


%width: 752, height: 480, fx: 364.61223237047539669, fy: 364.70254209576665971, cx: 385.60887400782206669, cy: 232.65626547906052224, model: 0, coeffs: [-0.31116718665000576, 0.08962547701688039, -0.00005212461326460, 0.00009844260968483, 0.00000000000000000


%% CALIBRATION
% Precalibrated extrinsic and intrinsics. These will be refined / estimated
% in the pipeline, but are used as good initial guesses.
focalLength = [441.998, 441.668];
principalPoint = [385.60887400782206669, 232.65626547906052224];
distortionCoefficients = [-0.297439888, 0.081949, -0.00005212461326460, 0.00009844260968483];

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
%cam0 = readtable(sprintf('%scam0/data.csv', datasetPath));
%imu0 = readtable(sprintf('%simu0/data.csv', datasetPath));

% these indices are the current
imuIndex = 1;
camIndex = 305; % this can be set to specify the start point.

camEnd = 2761;

while (camIndex <= camEnd)
    % skip every other frame
    %         if mod(camIndex, 2)
    %             camIndex = camIndex + 1;
    %             continue;
    %         end
    
    % Add a frame to vio here
    rawImage = double(imread(sprintf('%s/left/%06i.png', datasetPath, camIndex)));
    
    vio.addFrame(Frame(rawImage, maxIntensity, camIndex/25, cameraModel)); % add the frame
    
    camIndex = camIndex + 1;
    
    % draw
    if vio.graph.FrameContainer{end}.isKeyframe || true
        renderer.update(vio.graph);
    end
    %drawFrameGraph(vio.graph);
    %drawnow
    
end

close(renderer.vw);


% This is meant to test and evaluate my visual inertial slam method on all
% TUM VI datasets
clf;
datasetPath = 'datasets/dataset-room1_1024_16/mav0/';

bag = rosbag(datasetPath);
msgList = bag.MessageList;

vignette = imread('datasets/vignette.png', 'PNG'); % used as mask for feature tracking and selection

% Precalibrated extrinsic and intrinsics. These will be refined / estimated
% in the pipeline, but are used as good initial guesses.
focalLength = [380.81042871360756, 380.81194179427075];
principalPoint = [510.29465304840727, 514.3304630538506];
distortionCoefficients = [0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643];

accelBias = [-1.30318 -0.391441  0.380509]';
gyroBias = [0.0283122 0.00723077  0.0165292]';

gryoNoise = 0.00016;
accelNoise = 0.0028;

accelRandomWalk = 0.00086;
gyroRandomWalk = 2.2e-05;

accelScale = [1.00422, 0, 0;
                -7.82123e-05, 1.00136, 0;
                -0.0097745, -0.000976476, 0.970467];
gyroScale = [0.943611, 0.00148681, 0.000824366;
            0.000369694, 1.09413, -0.00273521;
             -0.00175252, 0.00834754, 1.01588];

T_camera0FromImu = [-0.99953783, 0.02917807, -0.0085308, 0.04709425;
    0.00752659, -0.03435493, -0.99938135, -0.04788273; 
    -0.0294531, -0.99898367, 0.03411944, -0.06972948; 
    0, 0, 0, 1];


% Create a camera model instance 
cameraModel = EquidistantCameraModel(distortionCoefficients, pi, focalLength, principalPoint, [1024;1024], 10000, vignette);

% Create a settings struct
settings = Settings(); % a default settings file for the VIO impl to use
settings.initial_T_camFromImu = T_camera0FromImu;
settings.initial_accelBias = accelBias;
settings.initial_gyroBias = gyroBias;
settings.initial_accelScale = accelScale;
settings.initial_gyroScale = gyroScale;

% Create an instance of a VIO
vio = VIO(settings);
tlast = 0;
for messageNumber = (1 : bag.NumMessages)
    msg = bag.readMessages(messageNumber);
    
    if (strcmp(char(msgList{messageNumber, 2}), '/cam0/image_raw'))
        % Add image to slam pipeline, scale to 255
        mat = double(msg{1}.readImage);
        
        vio = vio.addFrame(Frame(mat, msg{1}.Header.Stamp.seconds))
        
        imshow(mat, [0, 2^16])
    elseif (strcmp(char(msgList{messageNumber, 2}), '/imu0'))
        % Add IMU measurement to slam pipeline.
        
    end
end
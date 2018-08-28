% This is meant to test and evaluate my visual inertial slam method on all
% TUM VI datasets
clf;
datasetPath = 'datasets/dataset-room1_1024_16.bag';

bag = rosbag(datasetPath);
msgList = bag.MessageList;

% Precalibrated extrinsic and intrinsics. These will be refined / estimated
% in the pipeline, but are used as good initial guesses.
focalLength = [380.81042871360756, 380.81194179427075];
principalPoint = [510.29465304840727, 514.3304630538506];
distortionCoefficients = [0.010171079892421483, -0.010816440029919381, 0.005942781769412756, -0.001662284667857643];

T_camera0FromImu = [-0.99953783, 0.02917807, -0.0085308, 0.04709425;
    0.00752659, -0.03435493, -0.99938135, -0.04788273; 
    -0.0294531, -0.99898367, 0.03411944, -0.06972948; 
    0, 0, 0, 1];

for messageNumber = (1 : bag.NumMessages)
    msg = bag.readMessages(messageNumber);
    if (strcmp(char(msgList{messageNumber, 2}), '/cam0/image_raw'))
        % Add image to slam pipeline
        mat = msg{1}.readImage;
        imshow(mat)
    elseif (strcmp(char(msgList{messageNumber, 2}), '/imu0'))
        % Add IMU measurement to slam pipeline.
    end
end
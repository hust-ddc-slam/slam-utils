% clc;clear;close all;

% Specify the path to your ROS bag file
bagFilePath = '/home/larry/data/floor3_scout/2024-06-06-10-24-26.bag';
% bagFilePath = '/home/larry/data/floor3/10-290-line.bag';

% Load the ROS bag
bag = rosbag(bagFilePath);

% Select messages of type sensor_msgs/Imu
imuSelect = select(bag, 'Topic', '/ouster/imu');

% Read the IMU messages
imuMsgs = readMessages(imuSelect, 'DataFormat', 'struct');

% Initialize arrays to hold the data
timeStamps = zeros(length(imuMsgs), 1);
accelX = zeros(length(imuMsgs), 1);
accelY = zeros(length(imuMsgs), 1);
accelZ = zeros(length(imuMsgs), 1);
angVelX = zeros(length(imuMsgs), 1);
angVelY = zeros(length(imuMsgs), 1);
angVelZ = zeros(length(imuMsgs), 1);

% Extract the data from the messages
for i = 1:length(imuMsgs)
    timeStamps(i) = double(imuMsgs{i}.Header.Stamp.Sec) + double(imuMsgs{i}.Header.Stamp.Nsec) * 1e-9;
    accelX(i) = imuMsgs{i}.LinearAcceleration.X;
    accelY(i) = imuMsgs{i}.LinearAcceleration.Y;
    accelZ(i) = imuMsgs{i}.LinearAcceleration.Z;
    angVelX(i) = imuMsgs{i}.AngularVelocity.X;
    angVelY(i) = imuMsgs{i}.AngularVelocity.Y;
    angVelZ(i) = imuMsgs{i}.AngularVelocity.Z;
end

% Convert timestamps to relative time (starting from zero)
timeStamps = timeStamps - timeStamps(1);

% Plot the IMU data
figure;

% Plot linear acceleration
subplot(2, 1, 1);
plot(timeStamps, accelX, 'r', 'DisplayName', 'X-axis');
hold on;
plot(timeStamps, accelY, 'g', 'DisplayName', 'Y-axis');
plot(timeStamps, accelZ, 'b', 'DisplayName', 'Z-axis');
hold off;
title('Linear Acceleration');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
legend;

% Plot angular velocity
subplot(2, 1, 2);
plot(timeStamps, angVelX, 'r', 'DisplayName', 'X-axis');
hold on;
plot(timeStamps, angVelY, 'g', 'DisplayName', 'Y-axis');
plot(timeStamps, angVelZ, 'b', 'DisplayName', 'Z-axis');
hold off;
title('Angular Velocity');
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
legend;

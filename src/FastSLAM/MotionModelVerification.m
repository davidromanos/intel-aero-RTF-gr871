MocapFiles = dir('~/logs/*Mocap.txt');
MocapVelocityFiles = dir('~/logs/*MocapVelocity.txt');
MotionModelFiles = dir('~/logs/*MotionModel.txt');
%CameraFiles = dir('~/logs/*Camera.txt');
IntrinsicsFiles = dir('~/logs/*Intrinsics.txt');

mocap = csvread(['~/logs/' MocapFiles(end).name]); % open latest Mocap file
mocapVelocity = csvread(['~/logs/' MocapVelocityFiles(end).name]); % open latest Mocap Velocity file
motionModel = csvread(['~/logs/' MotionModelFiles(end).name]); % open latest Mocap Velocity file
%camera = csvread(['~/logs/' CameraFiles(end).name]);  % open latest Camera file
IntrinsicsFilename = ['~/logs/' IntrinsicsFiles(end).name];

t0 = min(mocap(1,1), camera(1,1));
mocap(:,1) = mocap(:,1) - t0;
mocapVelocity(:,1) = mocapVelocity(:,1) - t0;
%camera(:,1) = camera(:,1) - t0;

mocap = mocap(1:2:end,:); % discard every second measurement

tMoc = mocap(:,1);
tCam = camera(:,1);
pos = mocap(:,2:4);

x = mocap(:,2);
y = mocap(:,3);
z = mocap(:,4);
roll = mocap(:,5);
pitch = mocap(:,6);
yaw = mocap(:,7);

%%
tMot = motionModel(:,1);
dt = motionModel(:,2);
u = motionModel(:,3:6);
pose = motionModel(:,7:10);

figure(1);
subplot(3,1,1);
plot(tMoc, x, tMot, pose(:,1)+x(1));
title('X');
legend('Mocap', 'Dead reckoning');
subplot(3,1,2);
plot(tMoc, y, tMot, pose(:,2)+y(1));
title('Z');
legend('Mocap', 'Dead reckoning');
subplot(3,1,3);
plot(tMoc, z, tMot, pose(:,3)+z(1));
title('Z');
legend('Mocap', 'Dead reckoning');

figure(2);
plot(tMoc, rad2deg(yaw));

%%
figure(3);
plot(tMot, u(:,1));
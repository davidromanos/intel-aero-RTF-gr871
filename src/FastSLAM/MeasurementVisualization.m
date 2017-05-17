MocapFiles = dir('*Mocap.txt');
CameraFiles = dir('*Camera.txt');
IntrinsicsFiles = dir('*Intrinsics.txt');

mocap = csvread(MocapFiles(end).name); % open latest Mocap file
camera = csvread(CameraFiles(end).name);  % open latest Camera file
IntrinsicsFilename = IntrinsicsFiles(end).name;

t0 = min(mocap(1,1), camera(1,1));
mocap(:,1) = mocap(:,1) - t0;
camera(:,1) = camera(:,1) - t0;

mocap = mocap(1:2:end,:); % discard every second measurement

%% Read markers
markers = csvread('arucu_positions.txt');

%rowheaders = sprintf('%d ', 1:size(markers,1))
rowheaders = '-';
for (i = 1:size(markers,1))
    rowheaders = [rowheaders ' -'];
end

printmat(markers, 'My Matrix', rowheaders, 'ID X Y Z' );

header = {'ID', 'X', 'Y', 'Z'};
markersForDisplay = [header; num2cell(markers)];
disp(markersForDisplay);


%% Plot Mocap data
tMoc = mocap(:,1);
tCam = camera(:,1);
pos = mocap(:,2:4)';

figure(1);
%plot3(pos(1,:), pos(2,:), pos(3,:));
scatter3(pos(1,:), pos(2,:), pos(3,:), 10, tMoc);
axis equal;

dx = diff(pos(1,:));
dy = diff(pos(2,:));
dz = diff(pos(3,:));

dv = sqrt(dx.^2 + dy.^2 + dz.^2);
dt = diff(tMoc');

vel_ = dv ./ dt;
vel = smooth(vel_);
vel = smooth(vel);

figure(2);
plot(tMoc(1:end-1), vel_, tMoc(1:end-1), vel)

%% Calculate landmark position in a given frame (inverse measurement model)
% RGB intrinsics should be used
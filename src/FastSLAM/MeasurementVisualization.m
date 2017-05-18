format long g;

MocapFiles = dir('~/logs/*Mocap.txt');
CameraFiles = dir('~/logs/*Camera.txt');
IntrinsicsFiles = dir('~/logs/*Intrinsics.txt');

mocap = csvread(['~/logs/' MocapFiles(end).name]); % open latest Mocap file
camera = csvread(['~/logs/' CameraFiles(end).name]);  % open latest Camera file
IntrinsicsFilename = ['~/logs/' IntrinsicsFiles(end).name];

t0 = min(mocap(1,1), camera(1,1));
mocap(:,1) = mocap(:,1) - t0;
camera(:,1) = camera(:,1) - t0;

mocap = mocap(1:2:end,:); % discard every second measurement

tMoc = mocap(:,1);
tCam = camera(:,1);
pos = mocap(:,2:4);
roll = mocap(:,5);
pitch = mocap(:,6);
yaw = mocap(:,7);

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

%% Read intrinsics          
intrinFile = fopen(IntrinsicsFilename);
while (~feof(intrinFile))
    line = fgetl(intrinFile);
    s = split(line, ',');
    if (length(s) == 12)
        intrin.width = double(s(2));
        intrin.height = double(s(3));
        intrin.fx = double(s(4));
        intrin.ppx = double(s(5));
        intrin.fy = double(s(6));
        intrin.ppy = double(s(7));
        intrin.coeffs(1) = double(s(8));  
        intrin.coeffs(2) = double(s(9));  
        intrin.coeffs(3) = double(s(10));  
        intrin.coeffs(4) = double(s(11));  
        intrin.coeffs(5) = double(s(12));  
    end    
    if (contains(line, 'rgb'))        
        RGB = intrin;
    elseif (contains(line, 'depth'))
        Depth = intrin;
    end
end
fclose(intrinFile);

%% Pair measurements into measurement packs (pair timestamps)
tCamReduced = unique(tCam,'stable');

%% Plot Mocap data
figure(1);
%plot3(pos(:,1), pos(:,2), pos(:,3));
scatter3(pos(:,1), pos(:,2), pos(:,3), 10, tMoc);
axis equal;

dx = diff(pos(:,1));
dy = diff(pos(:,2));
dz = diff(pos(:,3));

dv = sqrt(dx.^2 + dy.^2 + dz.^2);
dt = diff(tMoc);

vel_ = dv ./ dt;
vel = smooth(vel_);
vel = smooth(vel);

figure(2);
plot(tMoc(1:end-1), vel_, tMoc(1:end-1), vel)

figure(3);
plot(tMoc, roll, tMoc, pitch, tMoc, yaw);

%% Unwrapping test
yaw_ = yaw;
yaw = zeros(size(yaw_));
yaw_old = yaw_(1);
yaw(1) = yaw_(1);

for (i = 2:length(tMoc))
    yaw(i) = uunwrap(yaw_(i), yaw(i-1), yaw_old);
    yaw_old = yaw_(i);    
end
figure(3);
plot(tMoc, yaw, tMoc, yaw_);

%% Plot drone positions and orientations
figure(1);
%plot3(pos(:,1), pos(:,2), pos(:,3));
scatter3(pos(:,1), pos(:,2), pos(:,3), 10, tMoc);
axis equal;

figure(1);
xlim([0.5 3.6]);
ylim([0.9 4]);
zlim([0 2]);

hold on;

for (i = 1:100:length(tMoc))
    R = rpy2tr(roll(i), pitch(i), yaw(i));
    R(1:3,4) = pos(i,:)';
    trplot(R, 'rgb', 'notext', 'noarrow', 'thick', 1, 'length', 0.2);
end
hold off;

%% Animated visualization
animVisual = false;
if (animVisual)
    figure(5);

    for (i = 1:10:length(tMoc))
        ax = gca;
        [a,b] = view;
        figure(5);    
        scatter3(pos(1:10:i,1), pos(1:10:i,2), pos(1:10:i,3), 10, tMoc(1:10:i));    
        axis equal;
        xlim([0.5 3.6]);
        ylim([0.9 4]);
        zlim([0 2]);
        view(ax, a, b);

        hold on;
        R = rpy2tr(roll(i), pitch(i), yaw(i));
        R(1:3,4) = pos(i,:)';
        trplot(R, 'rgb', 'notext', 'noarrow', 'thick', 1, 'length', 0.2);
        hold off;
        drawnow;
        %pause(tMoc(i+11) - tMoc(i));    
    end
end

%% Calculate landmark position in a given frame (inverse measurement model)
% RGB intrinsics should be used
i = 10; % pick camera measurement
t = tCamReduced(i);

iCam = find(tCam == t);
iMoc = find(tMoc > (t-0.005) & tMoc < (t+0.005));
if (length(iMoc) == 0)
    iMoc = find(tMoc > (t-0.01) & tMoc < (t+0.01));    
end
iMoc = iMoc(1);

pose = mocap(iMoc,2:7);
z = camera(iCam(1),3:5);
InverseMeasurementModel(pose, z, RGB)
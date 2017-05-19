MocapFiles = dir('~/logs/*Mocap.txt');
MocapVelocityFiles = dir('~/logs/*MocapVelocity.txt');
CameraFiles = dir('~/logs/*Camera.txt');
IntrinsicsFiles = dir('~/logs/*Intrinsics.txt');

mocap = csvread(['~/logs/' MocapFiles(end).name]); % open latest Mocap file
mocapVelocity = csvread(['~/logs/' MocapVelocityFiles(end).name]); % open latest Mocap Velocity file
camera = csvread(['~/logs/' CameraFiles(end).name]);  % open latest Camera file
IntrinsicsFilename = ['~/logs/' IntrinsicsFiles(end).name];

t0 = min(mocap(1,1), camera(1,1));
mocap(:,1) = mocap(:,1) - t0;
mocapVelocity(:,1) = mocapVelocity(:,1) - t0;
camera(:,1) = camera(:,1) - t0;

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

%% Read markers
markers = csvread('arucu_positions.txt'); % marker IDs in this TXT file are the numbers written on the paper (being the Aruco code + 1)
markers(:,2:4) = markers(:,2:4) / 100; % marker position are in cm

%rowheaders = sprintf('%d ', 1:size(markers,1))
rowheaders = '-';
for (i = 1:size(markers,1))
    rowheaders = [rowheaders ' -'];
end

%printmat(markers, 'My Matrix', rowheaders, 'ID X Y Z' );

header = {'ID', 'X', 'Y', 'Z'};
markersForDisplay = [header; num2cell(markers)];
%disp(markersForDisplay);

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
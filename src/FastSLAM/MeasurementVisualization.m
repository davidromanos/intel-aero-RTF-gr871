format long g;
run('LoadLatestLogs.m');

CamOffset = [0.09,-0.032,0.005]';

%CamOffset = [0.12,0.015,0.10]'; % this seem to be the best when checking here in MATLAB, but it does not give better results when running the FastSLAM system
%CamOffset = [0.09,-0.032,0.05]';

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

%% Plot actual landmark positions
hold on;
for i = 1:length(markers)
    text(markers(i,2),...
         markers(i,3),...
         markers(i,4),...
         num2str(markers(i,1)));
    scatter3(markers(i,2),...
             markers(i,3),...
             markers(i,4),'O');
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
clc;
i = 560; % pick camera measurement
t = tCamReduced(i);

iCam = find(tCam == t);
iMoc = find(tMoc > (t-0.005) & tMoc < (t+0.005));
if (length(iMoc) == 0)
    iMoc = find(tMoc > (t-0.01) & tMoc < (t+0.01));    
end
iMoc = iMoc(1);
measIdx = 1:length(iCam);

pose = mocap(iMoc,2:7)';
        hold on;
        R = rpy2tr(pose(4), pose(5), pose(6));
        R(1:3,4) = pose(1:3);
        trplot(R, 'color', 'red', 'notext', 'noarrow', 'thick', 3, 'length', 1);
        hold off;
  
zHat = [];        

for (measIdx_ = measIdx)
    DetectedID = camera(iCam(measIdx_),2)
    z = camera(iCam(measIdx_),3:5)
    marker = InverseMeasurementModel(pose, z, RGB,CamOffset);
    
    midx = find(markers(:,1) == DetectedID);
    if (length(midx) > 0)    
        l = markers(midx,2:4)';
        zHat = [zHat; MeasurementModel(pose, l, RGB,CamOffset)'];
    else
        zHat = [zHat; [-1,-1,-1]];
    end
    
    err = markers(:,2:4) - marker';
    errd = err(:,1).^2 + err(:,2).^2 + err(:,3).^2;
    [e idx] = min(errd);
    MatchingDistErr = errd(idx)
    MatchingID = markers(idx,1)
    if (DetectedID == MatchingID)
        disp('Detected and Calculated marker ID is matching');
    else
        disp('ERROR: Detected and Calculated marker ID does NOT match');
    end
end

figure(4);
scatter(camera(iCam,3), -camera(iCam,4), '*');
hold on;
scatter(zHat(:,1), -zHat(:,2), 'O');
text(zHat(:,1), -zHat(:,2), num2str(camera(iCam,2)));
hold off;
xlim([0 320]);
ylim([-240 0]);
legend('Camera measurement', 'Projected using known location');
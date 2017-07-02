close all
clear all

name = '';
if (length(name) == 0)
    Data = dir('~/catkin_ws/Data/t_*.m');
    timestamp = 0;
    idx = 0;
    for (i = 1:length(Data))
        if (Data(i).datenum > timestamp)
            idx = i;
            timestamp = Data(idx).datenum;
        end
    end
    run(['~/catkin_ws/Data/' Data(idx).name]);
    s1 = split(Data(idx).name, '_');
else
    run(['~/catkin_ws/Data/' name]);
    s1 = split(name, '_');
end

s2 = split(s1(2), '.');

t = eval(['t' char(s2(1))]);
run('LoadLatestLogs.m');

if (exist('~/catkin_ws/Data/landmarks.m') > 0)
    run('~/catkin_ws/Data/landmarks.m');
end

%% Extract FastSLAM path
estX = t.meanPath.Path(1,:);
estY = t.meanPath.Path(2,:);
estZ = t.meanPath.Path(3,:);
estYaw = t.meanPath.Path(4,:);
tFastSLAM = t.meanPath.Ts(:);

%% Plot drone positions and orientations
figure(1);
%scatter3(pos(:,1), pos(:,2), pos(:,3), 10, tMoc, 'o');
plot3(pos(:,1), pos(:,2), pos(:,3));
grid;
hold on;
scatter3(estX, estY, estZ, 10, tFastSLAM, '*');
hold off;
axis equal;

%% Plot landmarks
hold on 
i = 1;
markers_FastSLAM = [];
for j = 1:length(t.Particles(i).map.mean(1,:))
    text(t.Particles(i).map.mean(1,j),...
         t.Particles(i).map.mean(2,j),...
         t.Particles(i).map.mean(3,j),...
         num2str(t.Particles(i).map.identifier(j)));
    scatter3(t.Particles(i).map.mean(1,j),...
             t.Particles(i).map.mean(2,j),...
             t.Particles(i).map.mean(3,j),'*');
    
    marker = [t.Particles(i).map.identifier(j), t.Particles(i).map.mean(1,j), t.Particles(i).map.mean(2,j), t.Particles(i).map.mean(3,j)];
    markers_FastSLAM = [markers_FastSLAM; marker];
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

%% Draw connection lines between actual landmark positions and FastSLAM estimated positions
m0 = [];
r = [];
for i = 1:length(markers)
    ID = markers(i,1);
    idx = find(markers_FastSLAM(:,1) == ID);
    
    if (idx)    
        m0x = markers_FastSLAM(idx,2:4)';
        m1 = markers(i,2:4)';
        rx = m1-m0x;    
        
        m0 = [m0, m0x];
        r = [r, rx];
    end
end
hold on;
quiver3(m0(1,:), m0(2,:), m0(3,:), r(1,:), r(2,:), r(3,:), 0);
hold off;

%%
figure(2);
subplot(4,1,1);
plot(tMoc, pos(:,1), tFastSLAM, estX);
title('X');
legend('Mocap', 'FastSLAM');
ylabel('Meters');
subplot(4,1,2);
plot(tMoc, pos(:,2), tFastSLAM, estY);
title('Y');
legend('Mocap', 'FastSLAM');
ylabel('Meters');
subplot(4,1,3);
plot(tMoc, pos(:,3), tFastSLAM, estZ);
title('Z');
legend('Mocap', 'FastSLAM');
ylabel('Meters');
subplot(4,1,4);
plot(tMoc, rad2deg(yaw), tFastSLAM, rad2deg(estYaw));
title('Yaw');
legend('Mocap', 'FastSLAM');
ylabel('Degrees');
close all
clear all
Data = dir('~/catkin_ws/Data/*.m');
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
s2 = split(s1(2), '.');

t = eval(['t' char(s2(1))]);
run('LoadLatestLogs.m');

%% Extract FastSLAM path
x = t.meanPath.Path(1,:);
y = t.meanPath.Path(2,:);
z = t.meanPath.Path(3,:);
tFastSLAM = t.meanPath.Ts(:);

%% Plot drone positions and orientations
figure(1);
%scatter3(pos(:,1), pos(:,2), pos(:,3), 10, tMoc, 'o');
plot3(pos(:,1), pos(:,2), pos(:,3));
grid;
hold on;
scatter3(x, y, z, 10, tFastSLAM, '*');
hold off;
axis equal;
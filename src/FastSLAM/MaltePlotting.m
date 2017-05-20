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
plot3(pos(:,1), pos(:,2), pos(:,3));
grid;
hold on;

%%
figure
xlabel('x')
ylabel('y')
zlabel('z')
%xlim([-3,3]) 
%ylim([-3,3]) 
%zlim([-3,3]) 
grid
%
for i = 1%:length(t.Particles)
    hold on 
    for j = 1:length(t.Particles(i).map.mean(1,:))
        text(t.Particles(i).map.mean(1,j),...
             t.Particles(i).map.mean(2,j),...
             t.Particles(i).map.mean(3,j),...
             num2str(t.Particles(i).map.identifier(j)));
        scatter3(t.Particles(i).map.mean(1,j),...
                 t.Particles(i).map.mean(2,j),...
                 t.Particles(i).map.mean(3,j),'*');
    end
end
%
hold on 

x = t.meanPath.Path(1,:);
y = t.meanPath.Path(2,:);
z = t.meanPath.Path(3,:);
c = tFastSLAM;

cmap = colormap;
% change c into an index into the colormap
% min(c) -> 1, max(c) -> number of colors
c = round(1+(size(cmap,1)-1)*(c - min(c))/(max(c)-min(c)));
% make a blank plot
plot3(x,y,z,'linestyle','none')
% add line segments
for k = 1:(length(x)-1)
    line(x(k:k+1),y(k:k+1),z(k:k+1),'color',cmap(c(k),:),'Linewidth',2)
end




scatter3(x, y, z, 10, tFastSLAM, '*');
hold off;
axis equal;
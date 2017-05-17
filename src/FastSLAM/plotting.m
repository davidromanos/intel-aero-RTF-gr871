run('/home/malte/catkin_ws/Data/t_40.m')
t = t40;

figure
for i = 1:length(t.Particles)
    hold on 
    scatter3(t.Particles(i).map.mean(1,:),...
             t.Particles(i).map.mean(2,:),...
             t.Particles(i).map.mean(3,:),'*');
end

%
for i = 1:1%%length(t.Particles)
    hold on 
    scatter3(t.Particles(i).Path.Path(1,:),...
             t.Particles(i).Path.Path(2,:),...
             t.Particles(i).Path.Path(3,:));
end

hold on 
plot3(t.meanPath.Path(1,:),t.meanPath.Path(2,:),t.meanPath.Path(3,:))
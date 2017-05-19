close all
clear all
run('/home/malte/catkin_ws/Data/t_196.m')

t = t196;
%%
figure
xlabel('x')
ylabel('y')
zlabel('z')
%xlim([-3,3]) 
%ylim([-3,3]) 
%zlim([-3,3]) 
grid
%%
for i = 1:length(t.Particles)
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


%%
for i = 1:length(t.Particles)
    hold on 
    plot3(t.Particles(i).Path.Path(1,:),...
             t.Particles(i).Path.Path(2,:),...
             t.Particles(i).Path.Path(3,:),'b');
end
%%
hold on 
plot3(t.meanPath.Path(1,:),t.meanPath.Path(2,:),t.meanPath.Path(3,:),'r')


%% plot x,y,z
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(1,1:end-1),'r')
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(2,1:end-1),'r')
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(3,1:end-1),'r')

close all
clear all
%%
run('/home/malte/catkin_ws/Data/t_524.m')

t = t524;
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
c = t.meanPath.Ts(:);

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

%%
for i = 1:length(t.Particles)
    hold on 
    plot3(t.Particles(i).Path.Path(1,:),...
             t.Particles(i).Path.Path(2,:),...
             t.Particles(i).Path.Path(3,:),'b');
end


%% plot x,y,z
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(1,1:end-1),'r')
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(2,1:end-1),'r')
figure
plot(t.meanPath.Ts(1:end-1),t.meanPath.Path(3,1:end-1),'r')


%% sample rate
Hz = 1/(-1*mean(diff(t.meanPath.Ts)))
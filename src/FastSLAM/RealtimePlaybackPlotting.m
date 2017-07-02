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
    landmarks = map;
    clear map;
end

%% Extract FastSLAM path
estX = t.meanPath.Path(1,:);
estY = t.meanPath.Path(2,:);
estZ = t.meanPath.Path(3,:);
estYaw = t.meanPath.Path(4,:);
tFastSLAM = t.meanPath.Ts(:);

Plot3Denabled = true;
Plot2Denabled = true;

%% Plot 3D drone positions and orientations
figure(1);
if (Plot3Denabled)
    %figure(1);
    %close(1);
    %figure(1);
    for (i = 1:length(tMoc))
        iFS = find(tFastSLAM <= tMoc(i));
        [a iFScur] = max(tFastSLAM(iFS));
        iFScur = length(tFastSLAM) - iFS(iFScur) + 1;
    
        plot3(pos(1:i,1), pos(1:i,2), pos(1:i,3), 'k');
        grid;
        hold on;
        %scatter3(estX(iFS), estY(iFS), estZ(iFS), 10, tFastSLAM(iFS), '*');
        plot3(estX(iFS), estY(iFS), estZ(iFS), 'r');
        hold off;
        axis equal;
        
        % Plot all actual landmark locations
        hold on;
        for n = 1:size(markers,1)
            if (markers(n,1) ~= 4) % don't plot marker 4
            %text(markers(n,2),...
            %     markers(n,3),...
            %     markers(n,4),...
            %     num2str(markers(n,1)));
            scatter3(markers(n,2),...
                     markers(n,3),...
                     markers(n,4),'ks');
            end
        end
        hold off;      
        
        % Plot currently detected/estimated landmarks
        hold on;                
        for n = 2:size(landmarks(iFScur).mean,2)
            if (landmarks(iFScur).identifier(n) ~= 4) % don't plot marker 4
            text(landmarks(iFScur).mean(1,n)+0.05,...
                 landmarks(iFScur).mean(2,n)+0.05,...
                 landmarks(iFScur).mean(3,n)+0.05,...
                 num2str(landmarks(iFScur).identifier(n)));
            scatter3(landmarks(iFScur).mean(1,n),...
                     landmarks(iFScur).mean(2,n),...
                     landmarks(iFScur).mean(3,n),'rO');
            end
        end
        hold off;
        
        xlim([1 5.2]);
        ylim([0 5.3]);
        zlim([0 2.5]);
        s = sprintf('Time: %f', tMoc(i)); disp(s);
        drawnow;
        %pause(tMoc(i+1) - tMoc(i));
    end
end

%% Plot 2D drone positions and orientations
if (Plot2Denabled)
    figure(1);
    %close(1);
    %figure(1);
    for (i = 1:length(tMoc))
        iFS = find(tFastSLAM <= tMoc(i));
        
        subplot(4,1,1);
        plot(tMoc(1:i), pos(1:i,1), tFastSLAM(iFS), estX(iFS));
        title('X');
        legend('Mocap', 'FastSLAM');
        ylabel('Meters');
        xlim([tMoc(1) tMoc(end)]);
        ylim([1.5 3.5]);
        
        subplot(4,1,2);
        plot(tMoc(1:i), pos(1:i,2), tFastSLAM(iFS), estY(iFS));
        title('Y');
        legend('Mocap', 'FastSLAM');
        ylabel('Meters');
        xlim([tMoc(1) tMoc(end)]);
        ylim([2 4]);
        
        subplot(4,1,3);
        plot(tMoc(1:i), pos(1:i,3), tFastSLAM(iFS), estZ(iFS));
        title('Z');
        legend('Mocap', 'FastSLAM');
        ylabel('Meters');
        xlim([tMoc(1) tMoc(end)]);
        ylim([0 2]);        
        
        subplot(4,1,4);
        plot(tMoc(1:i), rad2deg(yaw(1:i)), tFastSLAM(iFS), rad2deg(estYaw(iFS)));
        title('Yaw');
        legend('Mocap', 'FastSLAM');
        ylabel('Degrees');        
        xlim([tMoc(1) tMoc(end)]);
        ylim([-10 10]);
        
        s = sprintf('Time: %f', tMoc(i)); disp(s);
        drawnow;
        %pause(3*(tMoc(i+1) - tMoc(i)));
    end
end
format long g;
run('LoadLatestLogs.m');

%% Plot Mocap data
figure(1);
%plot3(pos(:,1), pos(:,2), pos(:,3));
scatter3(pos(:,1), pos(:,2), pos(:,3), 10, tMoc);
axis equal;

dx = diff(pos(:,1));
zeroIdx = find(dx == 0);
dx(zeroIdx) = dx(zeroIdx-1); % velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity

dy = diff(pos(:,2));
zeroIdx = find(dy == 0);
dy(zeroIdx) = dy(zeroIdx-1); % velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity

dz = diff(pos(:,3));
zeroIdx = find(dz == 0);
dz(zeroIdx) = dz(zeroIdx-1); % velocity can not be zero, so we assume that it is because of no measurement, hence take the previous velocity

dv = sqrt(dx.^2 + dy.^2 + dz.^2);
dt = diff(tMoc);
td = tMoc(1:end-1);

xdot = dx ./ dt;
ydot = dy ./ dt;
zdot = dz ./ dt;

xdot_ = smooth(xdot);
ydot_ = smooth(ydot);
zdot_ = smooth(zdot);

%s = tf('s');
%omeg = 10;
%lpf = omeg / (s+omeg);
%xdotFilt = lsim(D, xdot, 0:mean(diff(td)):td(end));

D = designfilt('lowpassiir', 'FilterOrder', 2, ...
             'PassbandFrequency', 8, 'PassbandRipple', 0.5,...
             'SampleRate', round(1/mean(diff(td)), 0));
fvtool(D);

if (size(D.Coefficients,1) == 1)
    b = D.Coefficients(1,1:3);
    a = D.Coefficients(1,4:6);
    lpf = tf(b,a);
    bode(lpf);
elseif (size(D.Coefficients,1) == 2)
    b1 = D.Coefficients(1,1:3);
    a1 = D.Coefficients(1,4:6);
    b2 = D.Coefficients(2,1:3);
    a2 = D.Coefficients(2,4:6);

    lpf1 = tf(b1,a1,1/100);
    lpf2 = tf(b2,a2,1/100);
    lpf = lpf1*lpf2;
end

figure(11);
bode(lpf);

[b,a] = tfdata(minreal(lpf));
a = a{1};
b = b{1};

%figure(10);
grpdelay(D,2048);         

xdotFilt1 = filter(D, xdot);
%[b a] = tfdata(lpf);
%b = b{1};
%a = a{1};
%grpdelay(b,a,512,'whole');

%%
oldOutput = zeros((length(a)-1),1);
oldInput = zeros((length(b)-1),1);
xdotFilt2 = zeros(length(xdot),1);

for (i = 1:length(xdot))
    input = xdot(i);
    output = ( b(1)*input + oldInput'*b(2:end)' - oldOutput'*a(2:end)' ) / a(1);    
    
    for (j = length(oldOutput):-1:2)
        oldOutput(j) = oldOutput(j-1);
    end
    oldOutput(1) = output;
    for (j = length(oldInput):-1:2)
        oldInput(j) = oldInput(j-1);
    end    
    oldInput(1) = input;
    
    xdotFilt2(i) = output;
end

%%
vel_ = dv ./ dt;
vel = smooth(vel_);
vel = smooth(vel);

figure(2);
%plot(tMoc(1:end-1), vel_, tMoc(1:end-1), vel)
subplot(3,1,1);
plot(td, xdot, 'k', td, xdot_, 'r', td, xdotFilt1, 'y', td, xdotFilt2, 'm')
title('Xdot');
ylim([-1.5 1.5]);

subplot(3,1,2);
plot(td, ydot, 'k', td, ydot_, 'g')
title('Ydot');
ylim([-1.5 1.5]);

subplot(3,1,3);
plot(td, zdot, 'k', td, zdot_, 'b')
title('Zdot');
ylim([-1.5 1.5]);

%plot(tMoc(1:end-1), xdot, 'r', tMoc(1:end-1), ydot, 'g', tMoc(1:end-1), zdot, 'b')
%title('Mocap diffentiated (velocity)');
%legend('Xdot', 'Ydot', 'Zdot');
%ylim([-2 2]);

figure(9);
plot(td, xdot, td, xdotFilt2, mocapVelocity(:,1), mocapVelocity(:,2));

%%
figure(3);
plot(tMoc, x, 'r', tMoc, y, 'g', tMoc, z, 'b');
title('Mocap Position');
legend('X', 'Y', 'Z');

figure(4);
plot(tMoc, roll, tMoc, pitch, tMoc, yaw);
title('Mocap Orientation');
legend('Roll', 'Pitch', 'Yaw');

%% Generate x, y, z velocity in Drone frame
i = 1;
%R = rpy2tr(roll(i), pitch(i), yaw(i));
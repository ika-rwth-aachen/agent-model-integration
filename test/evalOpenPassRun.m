run = readmatrix('D:\home\git\SETLevel4to5\openpass\Binaries\Windows\debug.txt');
figure(1); % x,y plane
plot(run(:,2),run(:,3),'.');
daspect([1 1 1]);
title('x,y-plane');

figure(2); % acceleration over time
plot(run(:,1),run(:,4));
title('Acceleration');

figure(3); % speed over time
plot(run(:,1),run(:,5));
title('Speed');

figure(4); % yaw rate over time 
plot(run(:,1),run(:,6));
title('yaw rate');
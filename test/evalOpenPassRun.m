close all;
run = readmatrix('D:\home\git\SETLevel4to5\openpass\Binaries\Windows\debug.txt');
figure(1); 
subplot(2,2,1);% x,y plane
plot(run(:,2),run(:,3));
daspect([1 1 1]);
title('x,y-plane');

subplot(2,2,2); % acceleration over time
plot(run(:,1),run(:,4));
title('Acceleration');

subplot(2,2,3); % speed over time
plot(run(:,1),run(:,5));
title('Speed');

subplot(2,2,4); % yaw rate over time 
plot(run(:,1),run(:,6));
title('yaw rate');

figure(2);
subplot(2,2,1);
hold on;
plot(run(:,1),run(:,7));
plot(run(:,1),run(:,4)); 
title('a desired vs. actual');
subplot(2,2,2);
plot(run(:,1),run(:,8)); 
title('v local');
subplot(2,2,3);
plot(run(:,1),run(:,9));
title('v pred');

%% horizon
horizon = readmatrix('D:\home\git\SETLevel4to5\openpass\Binaries\Windows\horizon.txt');
i=209; % timestamp. 
a=i*32 + 1; b=(i+1)*32; 

% figure(5);
% subplot(2,1,1);
% plot(horizon(a:b,1),horizon(a:b,2));
% title(['x,y-horizon at t = ',num2str(i/10),'s']);
% daspect([1 1 1]);
% subplot(2,1,2);
% plot(horizon(a:b,3),horizon(a:b,4));
% title('kappa');


figure; plot(run(:,1),run(:,10))
figure; plot(run(:,1),run(:,11))
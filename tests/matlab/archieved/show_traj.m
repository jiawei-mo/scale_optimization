close all
D = load('/home/jiawei/Dropbox/IROS/eval/euroc/results/01.txt');
G = csvread('/home/jiawei/Dropbox/IROS/eval/euroc/poses/01.csv');
G(:,1) = G(:,1) / 1e9;
[gtp, dsvop, gtt, dsvot, gtvn, dsvovn, dsvo_diffo] = alignGT(D(:,1:4), G(:,1:4));
dsvo_diffn = abs(gtvn - dsvovn);
Method = 'DSVO';
scale_RMSE = sqrt(dsvo_diffn' * dsvo_diffn / size(dsvo_diffn,1)); 
scale_Median = median(dsvo_diffn);
direction_RMSE = sqrt(dsvo_diffo' * dsvo_diffo / size(dsvo_diffo,1));
direction_Median = median(dsvo_diffo);

figure('Name','Trajectory (Top View)')
plot3(dsvop(:,1), dsvop(:,2), dsvop(:,3), 'g-')
hold on
plot3(gtp(:,1), gtp(:,2), gtp(:,3), 'r-')
xlabel('x [m]');ylabel('y [m]');
legend('Truth', 'DSVO');
axis equal
view(90,90)

figure('Name', 'Position drift')
plot(gtt, vecnorm(dsvop'-gtp'), 'g-')
xlabel('Time [s]'); ylabel('position drift [m]');
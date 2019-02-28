T_g = load('/home/jiawei/Dropbox/IROS/eval/gazebo_grass/pose.txt');
T_d = load('/home/jiawei/Dropbox/IROS/eval/gazebo_grass/scale.txt');
T_s = load('/home/jiawei/Dropbox/IROS/eval/gazebo_grass/stereo.txt');

figure('Name','Trajectories (Top View)')
plot3(T_g(:,2), T_g(:,3), T_g(:,4), 'r-')
hold on
plot3(T_d(:,2), T_d(:,3), T_d(:,4), 'g-')
hold on
plot3(T_s(:,2), T_s(:,3), T_s(:,4), 'b-')
xlabel('x [m]');ylabel('y [m]');
legend('Truth', 'DSVO', 'StereoDSO');
axis equal
view(90,90)
% Reading in the data of stationary IMU
M = csvread('IMURest.csv');

time = M(:,1);
roll = M(:,2);
pitch = M(:,3);
yaw = M(:,4);

time = time(1:end-1);
time = (time - time(1)) * 1e-6;
roll = roll(1:end-1);
pitch = pitch(1:end-1);
yaw = yaw(1:end-1);


% Plotting 

subplot(3,1,1);
plot(time, roll);
ylabel('roll (degree)');
title('IMU at rest','fontsize',14);
grid;

subplot(3,1,2);
plot(time, pitch);
ylabel('pitch (degree)');
grid;

subplot(3,1,3);
plot(time, yaw);
xlabel('Time (sec)');
ylabel('Yaw (degree)');
grid;

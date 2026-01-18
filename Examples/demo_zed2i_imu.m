% demo_zed2i_imu.m
% IMU stream validation (sensor_msgs/Imu).

clearvars;
close all;
clc;

% Enable IMU via Name-Value if você já adotou isso no construtor:
% zed = ZED2i("enableImu", true);  % (se você ainda não expôs enableImu no construtor, use abaixo)
zed = ZED2i();
zed.pPar.enableImu = true;

cleanupObj = onCleanup(@() zed.rDisconnect());
zed.rConnect();

t_max = 10;
t = tic;

while toc(t) < t_max
    imu = zed.rGetImu();

    if zed.pFlag.HasImu
        w = imu.AngularVelocity;
        a = imu.LinearAcceleration;
        fprintf("w=[%.3f %.3f %.3f] rad/s | a=[%.3f %.3f %.3f] m/s^2\n", ...
            w(1), w(2), w(3), a(1), a(2), a(3));
    else
        fprintf("IMU not available. LastError: %s\n", string(zed.pFlag.LastError));
    end

    pause(0.05);
end

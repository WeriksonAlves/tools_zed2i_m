% demo_zed2i_calibration.m
% Fetch and print ZED2i camera calibration from ROS2 CameraInfo.

clearvars
clear all
close all
clc

zed = ZED2i();
zed.rConnect();

calib = zed.rGetCalibration();

disp('--- ZED2i Calibration ---');
disp(['Model: ' calib.DistortionModel]);
disp(['Size : ' num2str(calib.Width) ' x ' num2str(calib.Height)]);
disp('K ='); disp(calib.K);
disp('D ='); disp(calib.D.');

zed.rDisconnect();

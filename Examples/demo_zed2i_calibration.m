% demo_zed2i_calibration.m
% Simple demonstration of retrieving ZED2i calibration data using the "calibration" profile.
% This profile minimizes bandwidth by disabling all streams except calibration.
% Note: calibration data is typically static, so streaming at high rates is unnecessary.
%       In a real application, you might retrieve calibration once and then switch to a 
%       different profile for regular streaming.

%% Add project to path (root-based)
clearvars; close all; clc;
current_path = pwd;
root  = 'tools_zed2i_m';

idx = strfind(current_path, root);
if ~isempty(idx)
    rootPath = current_path(1:(idx(1) + numel(root) - 1));
    cd(rootPath);
    addpath(genpath(pwd));
    cd(current_path);
else
    addpath(genpath(current_path));
end

%% Create ZED2i object and guarantee proper cleanup
zed = ZED2i(0, "Profile", "calibration");
cleanupObj = onCleanup(@() safeDisconnect(zed)); %#ok<NASGU>

zed.rosConnect("UseCallbacks", false);

%% Retrieve calibration data
pause(1.0);  % gives ROS2 subscriber time to match publishers before receive()

data = zed.getSensorData();

if ~data.HasCalibration
    disp("Calibration not available.");
    disp("LastError: " + string(data.LastError));
    return;
end

calib = data.Calibration;
disp(calib);

if ~isempty(data.Intrinsics)
    disp("--- cameraIntrinsics ---");
    disp(data.Intrinsics);
end

function safeDisconnect(zed)
    try, zed.rosDisconnect(); catch, end
end

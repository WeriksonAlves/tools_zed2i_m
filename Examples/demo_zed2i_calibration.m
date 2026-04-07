% demo_zed2i_calibration.m
clearvars; close all; clc;

zed = ZED2i(0, "Profile", "calibration");
cleanupObj = onCleanup(@() safeDisconnect(zed)); %#ok<NASGU>

zed.rosConnect();

% In your new model, getSensorData is the commit point
zed.getSensorData();

if ~isfield(zed.pFlag, "HasCalibration") || ~zed.pFlag.HasCalibration
    disp("Calibration not available.");
    disp("LastError: " + string(zed.pFlag.LastError));
    return;
end

calib = zed.pData.Calibration;
disp(calib);

if isfield(zed.pData, "Intrinsics") && ~isempty(zed.pData.Intrinsics)
    disp("--- cameraIntrinsics ---");
    disp(zed.pData.Intrinsics);
end

function safeDisconnect(zed)
    try, zed.rosDisconnect(); catch, end
end

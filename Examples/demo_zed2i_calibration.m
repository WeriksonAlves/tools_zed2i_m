% demo_zed2i_calibration.m
% Fetch and display ZED2i camera calibration and MATLAB intrinsics from ROS2.

clearvars;
close all;
clc;

% -------------------------------------------------------------------------
% Create sensor object and guarantee proper cleanup
% -------------------------------------------------------------------------
zed = ZED2i();
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

% -------------------------------------------------------------------------
% Fetch calibration
% -------------------------------------------------------------------------
calib = zed.rGetCalibration();

disp('--- ZED2i Calibration ---');

if isempty(calib) || isempty(calib.K) || isempty(calib.Width) || isempty(calib.Height)
    disp('Calibration not available.');

    if isfield(zed.pFlag, "LastError") && ~isempty(zed.pFlag.LastError)
        disp(['LastError: ' char(zed.pFlag.LastError)]);
    end

    return;
end

% -------------------------------------------------------------------------
% Print calibration data
% -------------------------------------------------------------------------
disp(['Model: ' char(calib.DistortionModel)]);
disp(['Size : ' num2str(calib.Width) ' x ' num2str(calib.Height)]);

disp('K =');
disp(calib.K);

disp('D =');
if isempty(calib.D)
    disp([]);
else
    disp(calib.D(:).');
end

% -------------------------------------------------------------------------
% Fetch MATLAB camera intrinsics (if available)
% -------------------------------------------------------------------------
disp('--- MATLAB cameraIntrinsics ---');

try
    intr = zed.rGetIntrinsics();

    if isempty(intr)
        disp('cameraIntrinsics not available.');
    else
        disp(intr);
    end

catch excp
    disp('cameraIntrinsics could not be created.');
    disp(['Reason: ' excp.message]);
end

% demo_zed2i_calibration.m
% Fetch and print ZED2i camera calibration from ROS2 CameraInfo.

close all;
clc;

zed = ZED2i();
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

calib = zed.rGetCalibration();

disp('--- ZED2i Calibration ---');

if isempty(calib) || isempty(calib.K) || isempty(calib.Width) || isempty(calib.Height)
    disp('Calibration not available.');
    if isfield(zed.pFlag, "LastError") && ~isempty(zed.pFlag.LastError)
        disp(['LastError: ' char(zed.pFlag.LastError)]);
    end
    return;
end

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

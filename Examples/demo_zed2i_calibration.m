% demo_zed2i_calibration.m
% Fetch and display ZED2i camera calibration and MATLAB intrinsics from ROS2.

clearvars;
close all;
clc;

%% Add project to path (root-based)
PastaAtual = pwd;
PastaRaiz  = 'tools_zed2i_m';

idx = strfind(PastaAtual, PastaRaiz);
if ~isempty(idx)
    rootPath = PastaAtual(1:(idx(1) + numel(PastaRaiz) - 1));
    cd(rootPath);
    addpath(genpath(pwd));
    cd(PastaAtual);
else
    % Caso a pasta raiz não seja encontrada, ainda assim segue com o path atual
    addpath(genpath(PastaAtual));
end

%% Create sensor object and guarantee proper cleanup
zed = ZED2i();
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

%% Fetch calibration (CameraInfo + intrinsics)
disp('--- ZED2i Calibration ---');

calib = struct();
intr  = [];

try
    % Nova assinatura: [calib, intr] = rGetCalibration(zed)
    [calib, intr] = zed.rGetCalibration();
catch excp
    disp('Failed to retrieve calibration from ROS2 CameraInfo.');
    if isfield(zed.pFlag, "LastError") && ~isempty(zed.pFlag.LastError)
        disp(['LastError: ' char(zed.pFlag.LastError)]);
    end
    disp(['Reason: ' excp.message]);
    return;
end

%% Validate calibration
if isempty(calib) || ...
   ~isfield(calib, "K")      || isempty(calib.K)      || ...
   ~isfield(calib, "Width")  || isempty(calib.Width)  || ...
   ~isfield(calib, "Height") || isempty(calib.Height)

    disp('Calibration not available or incomplete.');
    if isfield(zed.pFlag, "LastError") && ~isempty(zed.pFlag.LastError)
        disp(['LastError: ' char(zed.pFlag.LastError)]);
    end
    return;
end

%% Print calibration data
disp(['Model: ' char(calib.DistortionModel)]);
disp(['Size : ' num2str(calib.Width) ' x ' num2str(calib.Height)]);

disp('K =');
disp(calib.K);

disp('D =');
if ~isfield(calib, "D") || isempty(calib.D)
    disp([]);
else
    disp(calib.D(:).');
end

%% Print MATLAB camera intrinsics (if available)
disp('--- MATLAB cameraIntrinsics ---');

if isempty(intr)
    disp('cameraIntrinsics not available (e.g., missing Computer Vision Toolbox).');
else
    disp(intr);
end

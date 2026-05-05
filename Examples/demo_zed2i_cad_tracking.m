% demo_zed2i_cad_tracking.m
% Real-time ZED2i CAD pose visualization using ROS2 odometry.
%
% This demo tracks the ZED2i pose in real time, updates the CAD model
% according to the current odometry, and logs the camera trajectory.

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


%% Demo configuration
maxTimeSec = 60;
targetPlotHz = 30;
maxTrajectoryPoints = 3000;

% Visualization options
followCamera = true;
followRadius = 0.5;

initialAxisRadius = 0.50;

cadOffset = [0 0 0];
cadShowLines = false;
cadUniformColor = [0.2 0.2 0.8];

% ROS2 receive mode.
% false: direct receive/fetch mode, deterministic for validation.
% true : callback/cache mode, useful for asynchronous operation.
useCallbacks = false;

%% Create ZED2i object
zed = ZED2i(0, "Profile", "live");

% This demo only needs odometry/pose.
zed.setEnabled("enableImu", false);
zed.setEnabled("enablePose", true);
zed.setEnabled("enablePointCloud", false);

zed.setStreamHz("image", 0);
zed.setStreamHz("depth", 0);
zed.setStreamHz("imu", 0);
zed.setStreamHz("pose", targetPlotHz);
zed.setStreamHz("pcd", 0);
zed.setStreamHz("calib", 0);

cleanupObj = onCleanup(@() safeDisconnect(zed)); %#ok<NASGU>

%% Connect to ROS2
zed.rosConnect("UseCallbacks", useCallbacks);

%% State log buffers
timeLog = zeros(0, 1);
positionLog = zeros(0, 3);
orientationLog = zeros(0, 3);

trajectory = zeros(0, 3);

%% Figure setup
hFig = figure( ...
    "Name", "ZED2i CAD Pose Tracking", ...
    "NumberTitle", "off", ...
    "Color", "w" ...
);

ax = axes("Parent", hFig);
hold(ax, "on");
grid(ax, "on");
axis(ax, "equal");
view(ax, 3);

xlabel(ax, "X [m]");
ylabel(ax, "Y [m]");
zlabel(ax, "Z [m]");
title(ax, "ZED2i Real-Time CAD Pose Tracking");

xlim(ax, [-initialAxisRadius initialAxisRadius]);
ylim(ax, [-initialAxisRadius initialAxisRadius]);
zlim(ax, [-initialAxisRadius initialAxisRadius]);

axes(ax); %#ok<LAXES>

%% Create CAD model once at origin
zed.pPos.X = zeros(12, 1);
zed.mCADplot(cadShowLines);

% Optional uniform color.
try
    zed.mCADcolor(cadUniformColor);
catch
    % Keep material colors if uniform color fails.
end

%% Trajectory graphics
hTrajectory = plot3( ...
    ax, ...
    NaN, NaN, NaN, ...
    "LineWidth", 1.5 ...
);

hCurrentPoint = plot3( ...
    ax, ...
    NaN, NaN, NaN, ...
    "o", ...
    "MarkerSize", 6, ...
    "LineWidth", 1.5 ...
);

%% Main loop
fprintf("Starting ZED2i CAD pose tracking demo...\n");
fprintf("Close the figure window to stop.\n");

tStart = tic;
tPlot = tic;
frameCount = 0;

while ishandle(hFig) && toc(tStart) < maxTimeSec
    if toc(tPlot) < 1 / targetPlotHz
        pause(0.001);
        continue;
    end
    tPlot = tic;

    data = zed.getSensorData();

    if ~data.Connected
        fprintf("[ZED2i] Disconnected. LastError: %s\n", ...
            string(data.LastError));
        break;
    end

    if ~data.HasPose
        if mod(frameCount, targetPlotHz) == 0
            fprintf("[ZED2i] Waiting for pose. LastError: %s\n", ...
                string(data.LastError));
        end

        drawnow limitrate;
        frameCount = frameCount + 1;
        continue;
    end

    pose = data.Pose;

    if ~isValidPose_(pose)
        if mod(frameCount, targetPlotHz) == 0
            fprintf("[ZED2i] Invalid pose data received.\n");
        end

        drawnow limitrate;
        frameCount = frameCount + 1;
        continue;
    end

    position = double(pose.Position(:).');
    eulerRPY = double(pose.OrientationEuler(:).');

    elapsedSec = toc(tStart);

    % Log state.
    timeLog(end + 1, 1) = elapsedSec; %#ok<SAGROW>
    positionLog(end + 1, :) = position; %#ok<SAGROW>
    orientationLog(end + 1, :) = eulerRPY; %#ok<SAGROW>

    % Update CAD pose.
    zed.mCADsetPose(position, eulerRPY, ...
        "Offset", cadOffset, ...
        "ShowLines", cadShowLines);

    % Update trajectory buffer.
    trajectory(end + 1, :) = position; %#ok<SAGROW>

    if size(trajectory, 1) > maxTrajectoryPoints
        trajectory = trajectory((end - maxTrajectoryPoints + 1):end, :);
    end

    % Update trajectory graphics.
    set(hTrajectory, ...
        "XData", trajectory(:, 1), ...
        "YData", trajectory(:, 2), ...
        "ZData", trajectory(:, 3));

    set(hCurrentPoint, ...
        "XData", position(1), ...
        "YData", position(2), ...
        "ZData", position(3));

    title(ax, sprintf( ...
        "ZED2i Pose | x=%.2f y=%.2f z=%.2f | roll=%.1f pitch=%.1f yaw=%.1f deg", ...
        position(1), position(2), position(3), ...
        rad2deg(eulerRPY(1)), ...
        rad2deg(eulerRPY(2)), ...
        rad2deg(eulerRPY(3))));

    if followCamera
        keepCameraCentered_(ax, position, followRadius);
    end

    drawnow limitrate nocallbacks;

    frameCount = frameCount + 1;
end

fprintf("Demo finished.\n");

%% Optional summary
if ~isempty(positionLog)
    fprintf("\nPose samples collected: %d\n", size(positionLog, 1));
    fprintf("Initial position: [%.3f %.3f %.3f] m\n", positionLog(1, :));
    fprintf("Final position:   [%.3f %.3f %.3f] m\n", positionLog(end, :));

    displacement = positionLog(end, :) - positionLog(1, :);
    fprintf("Displacement:     [%.3f %.3f %.3f] m\n", displacement);
end

%% Cleanup
clear cleanupObj zed;

%% ------------------------------------------------------------------------
% Local helper functions
% -------------------------------------------------------------------------
function safeDisconnect(zed)
    try
        zed.rosDisconnect();
    catch
    end
end

function tf = isValidPose_(pose)
    tf = false;

    if ~isstruct(pose)
        return;
    end

    if ~isfield(pose, "Position") || ~isfield(pose, "OrientationEuler")
        return;
    end

    position = double(pose.Position(:).');
    eulerRPY = double(pose.OrientationEuler(:).');

    if numel(position) ~= 3 || numel(eulerRPY) ~= 3
        return;
    end

    if any(~isfinite(position)) || any(~isfinite(eulerRPY))
        return;
    end

    tf = true;
end

function keepCameraCentered_(ax, position, radius)
    xlim(ax, [position(1) - radius, position(1) + radius]);
    ylim(ax, [position(2) - radius, position(2) + radius]);
    zlim(ax, [position(3) - radius, position(3) + radius]);
end
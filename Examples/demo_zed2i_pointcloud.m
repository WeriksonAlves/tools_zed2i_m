% demo_zed2i_pointcloud.m
% Simple demonstration of retrieving PointCloud2 data from the ZED2i.
%
% Operation modes:
%   1) "static": captures one current point cloud frame and displays it.
%   2) "live"  : displays the current point cloud during t_max seconds.
%
% This demo uses getSensorData() as the high-level acquisition API.
% Direct rosGetPointCloud() calls are intentionally avoided here because
% rosGet* methods are decode-only helpers, while getSensorData() is the
% single commit point for class state.

%% Add project to path (root-based)
clearvars; close all; clc;

current_path = pwd;
root = 'tools_zed2i_m';

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
% Choose:
%   "static" -> show a single current point cloud frame
%   "live"   -> update the current point cloud during t_max seconds
demoMode = "static";

t_max = 15;              % [s] live visualization duration
target_fps_cloud = 5;    % [Hz] visualization/update loop rate
pointCloudHz = 5;        % [Hz] point cloud acquisition rate
maxPointsToShow = 1e4;   % max number of points rendered by scatter3

%% Create ZED2i object and guarantee proper cleanup
zed = ZED2i(0, "Profile", "live");

% Keep only the point cloud stream enabled for this demo.
zed.setEnabled("enableImu", false);
zed.setEnabled("enablePose", false);
zed.setEnabled("enablePointCloud", true);

% Disable streams not used by this demo.
zed.setStreamHz("image", 0);
zed.setStreamHz("depth", 0);
zed.setStreamHz("imu", 0);
zed.setStreamHz("pose", 0);
zed.setStreamHz("calib", 0);

% PointCloud2 is heavy, so keep acquisition conservative.
zed.setStreamHz("pcd", pointCloudHz);

cleanupObj = onCleanup(@() safeDisconnect(zed)); %#ok<NASGU>

% Use direct receive/fetch mode for deterministic validation.
% Callback/cache mode can be validated separately.
zed.rosConnect("UseCallbacks", false);

%% Wait for one valid point cloud
fprintf("Waiting for first valid point cloud...\n");

[pc, ok, err] = waitForPointCloud(zed, t_max, target_fps_cloud);

if ~ok
    fprintf("Could not get initial point cloud. LastError: %s\n", string(err));
    return;
end

fprintf("First PointCloud received via getSensorData().\n");
fprintf("Initial point cloud: %d points.\n", size(pc.XYZ, 1));

%% Create initial visualization
[hFig, hScat] = createPointCloudFigure(pc, maxPointsToShow, demoMode);

switch lower(demoMode)
    case "static"
        title(sprintf("ZED2i PointCloud | static frame | pts = %d", ...
            size(pc.XYZ, 1)));
        fprintf("Static point cloud displayed.\n");

    case "live"
        runLivePointCloudPreview( ...
            zed, hFig, hScat, t_max, target_fps_cloud, maxPointsToShow);

    otherwise
        safeDisconnect(zed);
        clear cleanupObj zed;
        error("ZED2i:Demo:InvalidMode", ...
            "Invalid demoMode '%s'. Use 'static' or 'live'.", demoMode);
end

% Explicit cleanup is needed because this file is a script, not a function.
safeDisconnect(zed);
clear cleanupObj zed;

%% ------------------------------------------------------------------------
% Local helper functions
% -------------------------------------------------------------------------

function [pc, ok, err] = waitForPointCloud(zed, timeoutSec, targetRateHz)
%waitForPointCloud Wait until getSensorData() returns a valid point cloud.

    pc = zed.sEmptyPointCloud();
    ok = false;
    err = "";

    t0 = tic;
    tc = tic;

    while toc(t0) < timeoutSec
        if toc(tc) < 1 / targetRateHz
            pause(0.01);
            continue;
        end
        tc = tic;

        data = zed.getSensorData();

        if ~data.Connected
            err = "Disconnected. LastError: " + string(data.LastError);
            return;
        end

        if data.HasPointCloud ...
                && isfield(data.PointCloud, "XYZ") ...
                && ~isempty(data.PointCloud.XYZ)
            pc = data.PointCloud;
            ok = true;
            return;
        end

        err = data.LastError;
        fprintf("Waiting for PointCloud...\n");
    end
end

function [hFig, hScat] = createPointCloudFigure(pc, maxPointsToShow, modeName)
%createPointCloudFigure Create a scatter3 visualization for one point cloud.

    xyzVis = subsamplePointCloud(pc.XYZ, maxPointsToShow);

    hFig = figure( ...
        'Name', sprintf('ZED2i PointCloud - %s', modeName), ...
        'NumberTitle', 'off');

    hScat = scatter3( ...
        xyzVis(:, 1), ...
        xyzVis(:, 2), ...
        xyzVis(:, 3), ...
        1, ...
        xyzVis(:, 3), ...
        '.');

    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    axis equal;

    % Fixed view limits for stable visualization.
    xlim([-1 3]);
    ylim([-2 2]);
    zlim([-2 2]);
    axis manual;

    colormap turbo;
    colorbar;
    title(sprintf("ZED2i PointCloud | %s", modeName));

    drawnow;
end

function runLivePointCloudPreview( ...
    zed, hFig, hScat, tMax, targetRateHz, maxPointsToShow)
%runLivePointCloudPreview Update the displayed point cloud during tMax seconds.

    fprintf("Starting live point cloud preview for %.1f s...\n", tMax);

    tStart = tic;
    tc = tic;
    frameCount = 0;

    while ishandle(hFig) && toc(tStart) < tMax
        if toc(tc) < 1 / targetRateHz
            pause(0.01);
            continue;
        end
        tc = tic;

        data = zed.getSensorData();

        if ~data.Connected
            fprintf("ZED2i disconnected during live preview. LastError: %s\n", ...
                string(data.LastError));
            break;
        end

        if ~(data.HasPointCloud ...
                && isfield(data.PointCloud, "XYZ") ...
                && ~isempty(data.PointCloud.XYZ))
            if mod(frameCount, 10) == 0
                fprintf("PointCloud not available in this frame. LastError: %s\n", ...
                    string(data.LastError));
            end
            frameCount = frameCount + 1;
            continue;
        end

        xyz = data.PointCloud.XYZ;
        xyzVis = subsamplePointCloud(xyz, maxPointsToShow);

        if ishandle(hScat)
            set(hScat, ...
                'XData', xyzVis(:, 1), ...
                'YData', xyzVis(:, 2), ...
                'ZData', xyzVis(:, 3), ...
                'CData', xyzVis(:, 3));
        end

        title(sprintf( ...
            "ZED2i PointCloud | live | t = %.1f s | pts = %d", ...
            toc(tStart), size(xyz, 1)));

        drawnow limitrate nocallbacks;

        frameCount = frameCount + 1;
    end

    fprintf("Live point cloud preview finished.\n");
end

function xyzVis = subsamplePointCloud(xyz, maxPoints)
%subsamplePointCloud Randomly subsample point cloud for visualization.

    if isempty(xyz)
        xyzVis = zeros(0, 3, "single");
        return;
    end

    nPoints = size(xyz, 1);

    if nPoints > maxPoints
        idxSub = randperm(nPoints, maxPoints);
        xyzVis = xyz(idxSub, :);
    else
        xyzVis = xyz;
    end
end

function safeDisconnect(zed)
%safeDisconnect Best-effort ROS2 cleanup.

    try
        zed.rosDisconnect();
    catch
    end
end
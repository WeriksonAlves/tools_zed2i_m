% demo_zed2i_rgbd.m
% Simple demonstration of retrieving RGB-D data from the ZED2i.
% This demo uses the "minimal" profile and explicitly enables only RGB and Depth.
% It uses direct fetch mode instead of callbacks to provide deterministic
% behavior during basic validation.

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

%% Create ZED2i object and guarantee proper cleanup
zed = ZED2i(0, "Profile", "minimal");

% Keep only the point cloud stream enabled for this demo.
zed.setEnabled("enableImu", false);
zed.setEnabled("enablePose", false);
zed.setEnabled("enablePointCloud", false);

% Disable streams not used by this demo.
zed.setStreamHz("image", 60);
zed.setStreamHz("depth", 60);
zed.setStreamHz("imu", 0);
zed.setStreamHz("pose", 0);
zed.setStreamHz("calib", 0);
zed.setStreamHz("pcd", 0);

cleanupObj = onCleanup(@() safeDisconnect(zed)); %#ok<NASGU>

% Use direct receive/fetch mode for deterministic validation.
% Callback/cache mode can be validated separately.
zed.rosConnect("UseCallbacks", false);

%% Visualization and timing parameters
t_max      = 15;  % [s] total demo time
target_fps = 15;   % desired preview FPS

%% Pre-create figure and graphics objects
hFig = figure('Name', 'ZED2i RGB-D Preview', 'NumberTitle', 'off');

ax1 = subplot(1, 2, 1);
hImg = imshow(zeros(360, 640, 3, 'uint8'), 'Parent', ax1);
title(ax1, 'RGB');

ax2 = subplot(1, 2, 2);
hDepth = imagesc(zeros(360, 640, 'single'), 'Parent', ax2);
axis(ax2, 'image');
axis(ax2, 'off');
colormap(ax2, turbo);
cb = colorbar(ax2);
cb.Label.String = 'Depth (m)';
title(ax2, 'Depth');

% Initial depth scaling in meters
lo = 0.3;
hi = 6.0;
set(ax2, 'CLim', [lo hi]);

%% Main loop
tc = tic;
t  = tic;
frameCount = 0;

while ishandle(hFig) && toc(t) < t_max
    if toc(tc) <= 1 / target_fps
        pause(0.001);
        continue;
    end

    tc = tic;
    cycleTic = tic;

    data = zed.getSensorData();

    if mod(frameCount, 30) == 0
        fprintf('\n--- getSensorData debug ---\n');
        fprintf('Connected: %d\n', data.Connected);
        fprintf('HasImage: %d\n', data.HasImage);
        fprintf('HasDepth: %d\n', data.HasDepth);
        fprintf('ImageFps: %.2f\n', data.Metrics.ImageFps);
        fprintf('DepthFps: %.2f\n', data.Metrics.DepthFps);

        err = char(data.LastError);
        if strlength(string(err)) > 180
            err = [err(1:180), '...'];
        end
        fprintf('LastError: %s\n', err);
    end

    if ~data.Connected
        if ~isempty(data.LastError)
            fprintf("[ZED2i] Disconnected: %s\n", char(data.LastError));
        end
        break;
    end

    % -------------------- RGB --------------------
    if data.HasImage && ~isempty(data.Image)
        set(hImg, 'CData', data.Image);
        title(ax1, sprintf('RGB | FPS: %.1f | Drops: %d', ...
            data.Metrics.ImageFps, data.Metrics.ImageDrops));
    else
        title(ax1, 'RGB (no data)');
    end

    % -------------------- Depth --------------------
    if data.HasDepth && ~isempty(data.Depth)
        set(hDepth, 'CData', data.Depth);

        if mod(frameCount, 10) == 0
            if isfield(data, "DepthMask") && ~isempty(data.DepthMask)
                v = data.Depth(data.DepthMask);
            else
                v = data.Depth(isfinite(data.Depth) & data.Depth > 0);
            end

            if ~isempty(v)
                if numel(v) > 50000
                    idxSample = round(linspace(1, numel(v), 50000));
                    v = v(idxSample);
                end

                lo = prctile(v, 2);
                hi = prctile(v, 98);

                if hi <= lo
                    hi = lo + 1e-3;
                end

                set(ax2, 'CLim', [lo hi]);
            end
        end

        title(ax2, sprintf('Depth | Range: [%.2f, %.2f] m | FPS: %.1f', ...
            lo, hi, data.Metrics.DepthFps));
    else
        title(ax2, 'Depth (no data)');
    end

    drawnow limitrate nocallbacks;

    frameCount = frameCount + 1;

    if mod(frameCount, 30) == 0
        fprintf('Cycle time: %.3f s\n', toc(cycleTic));
    end
end

% Explicit cleanup is needed because this file is a script, not a function.
safeDisconnect(zed);
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
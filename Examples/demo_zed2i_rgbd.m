% demo_zed2i_rgbd.m
% RGB + Depth preview.

clearvars;
close all;
clc;

% -------------------------------------------------------------------------
% Create ZED2i object and guarantee proper cleanup
% -------------------------------------------------------------------------
zed = ZED2i();
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

% -------------------------------------------------------------------------
% Visualization and timing parameters
% -------------------------------------------------------------------------
t_max      = 30;   % [s] total demo time
target_fps = 15;   % desired preview FPS

% -------------------------------------------------------------------------
% Pre-create figure and graphics objects (NO clf/subplot/colorbar per frame)
% -------------------------------------------------------------------------
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

% -------------------------------------------------------------------------
% Main loop
% -------------------------------------------------------------------------
tc = tic;          % frame pacing timer
t  = tic;          % total demo timer
frameCount = 0;

while ishandle(hFig) && toc(t) < t_max

    % Simple FPS limiter
    if toc(tc) < 1 / target_fps
        pause(0.001);
        continue;
    end
    tc = tic;

    cycleTic = tic;

    ok = zed.rGrab();
    if ~ok
        % Se nenhum stream foi atualizado, segue para próxima iteração
        continue;
    end

    % -------------------- RGB --------------------
    img = zed.rGetImage();
    if ~isempty(img)
        set(hImg, 'CData', img);
        title(ax1, sprintf('RGB | FPS: %.1f | Drops: %d', ...
            zed.pData.Metrics.ImageFps, zed.pData.Metrics.ImageDrops));
    end

    % -------------------- Depth --------------------
    [depth, mask] = zed.rGetDepth(); %#ok<NASGU>
    if ~isempty(depth)
        set(hDepth, 'CData', depth);

        % Update contrast every 10 frames (cheap and stable)
        if mod(frameCount, 10) == 0
            v = depth(isfinite(depth) & depth > 0);
            if ~isempty(v)
                lo = prctile(v, 2);
                hi = prctile(v, 98);
                if hi <= lo
                    hi = lo + 1e-3;
                end
                set(ax2, 'CLim', [lo hi]);
            end
        end

        title(ax2, sprintf('Depth | Range: [%.2f, %.2f] m | FPS: %.1f', ...
            lo, hi, zed.pData.Metrics.DepthFps));
    end

    drawnow limitrate nocallbacks;

    frameCount = frameCount + 1;

    % Optional: log de tempo de ciclo (a cada 30 frames, para não poluir)
    if mod(frameCount, 30) == 0
        fprintf('Cycle time (avg over last frame): %.3f s\n', toc(cycleTic));
    end
end

% rDisconnect será chamado automaticamente por cleanupObj

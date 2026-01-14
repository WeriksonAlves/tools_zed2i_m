% demo_main_fast.m

clearvars
close all
clc

zed = ZED2i();
zed.rConnect();

t_max = 30;
target_fps = 15;

% --- Pre-create figure and graphics objects (NO clf/subplot/colorbar per frame) ---
hFig = figure('Name','ZED2i RGB-D Preview','NumberTitle','off');

ax1 = subplot(1,2,1);
hImg = imshow(zeros(360,640,3,'uint8'), 'Parent', ax1);
title(ax1, 'RGB');

ax2 = subplot(1,2,2);
hDepth = imagesc(zeros(360,640,'single'), 'Parent', ax2);
axis(ax2, 'image'); axis(ax2, 'off');
colormap(ax2, turbo);
cb = colorbar(ax2);
cb.Label.String = 'Depth (m)';
title(ax2, 'Depth');

% Initial depth scaling
lo = 0.3;
hi = 6.0;
set(ax2, 'CLim', [lo hi]);

tc = tic;
t = tic;
frameCount = 0;

while ishandle(hFig) && toc(t) < t_max
    if toc(tc) < 1 / target_fps
        pause(0.001);
        continue;
    end
    tc = tic;

    t1 = tic;

    ok = zed.rGrab();
    if ~ok
        continue;
    end

    img = zed.rGetImage();
    [depth, mask] = zed.rGetDepth(); %#ok<NASGU>

    if ~isempty(img)
        set(hImg, 'CData', img);
        title(ax1, sprintf('RGB | FPS: %.1f | Drops: %d', ...
            zed.pData.Metrics.ImageFps, zed.pData.Metrics.ImageDrops));
    end

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

    drawnow limitrate nocallbacks

    frameCount = frameCount + 1;
    fprintf('Cycle time: %.3f s\n', toc(t1));
end

zed.rDisconnect();

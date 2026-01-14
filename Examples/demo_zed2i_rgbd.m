% demo_zed2i_rgbd.m
% RGB + Depth preview (minimal, stable).

clearvars
clear all
close all
clc

zed = ZED2i();
zed.rConnect();

hFig = figure('Name', 'ZED2i RGB-D Preview', 'NumberTitle', 'off');
t0 = tic;
seconds = 30;

while ishandle(hFig) && toc(t0) < seconds
    ok = zed.rGrab();
    if ~ok
        pause(0.01);
        continue;
    end

    img = zed.rGetImage();
    [depth, mask] = zed.rGetDepth();

    clf;

    subplot(1,2,1);
    if ~isempty(img)
        imshow(img);
        title(sprintf('RGB | FPS: %.1f | Drops: %d', ...
            zed.pData.Metrics.ImageFps, zed.pData.Metrics.ImageDrops));
    else
        axis off;
        text(0.1, 0.5, 'No RGB');
    end

    subplot(1,2,2);
    if ~isempty(depth)

        d = depth(:);
        d = d(isfinite(d) & d > 0);

        if ~isempty(d)
            lo = prctile(d, 2);
            hi = prctile(d, 98);

            imagesc(depth, [lo hi]);   % linear scaling, in meters
            axis image off;
            colormap(gca, turbo);

            cb = colorbar;
            cb.Label.String = 'Depth (m)';

            title(sprintf('Depth | Range: [%.2f, %.2f] m | FPS: %.1f', ...
                lo, hi, zed.pData.Metrics.DepthFps));
        else
            axis off;
            text(0.1, 0.5, 'Depth has no valid values');
        end

    else
        axis off;
        text(0.1, 0.5, 'No Depth');
    end



    drawnow;
end

zed.rDisconnect();

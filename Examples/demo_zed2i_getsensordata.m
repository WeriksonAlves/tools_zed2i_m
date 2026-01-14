% demo_zed2i_getsensordata.m
% Lab-style aggregated access via rGetSensorData.

clearvars
clear all
close all
clc

zed = ZED2i();
zed.rConnect();

hFig = figure('Name', 'ZED2i rGetSensorData Demo', 'NumberTitle', 'off');
t0 = tic;
seconds = 30;

while ishandle(hFig) && toc(t0) < seconds
    s = zed.rGetSensorData();

    clf;

    subplot(1,2,1);
    if s.HasImage
        imshow(s.Image);
        title(sprintf('RGB | FPS: %.1f | Drops: %d', ...
            s.Metrics.ImageFps, s.Metrics.ImageDrops));
    else
        axis off;
        text(0.1, 0.5, 'No RGB');
    end

    subplot(1,2,2);
    if s.HasDepth && ~isempty(s.Depth)
        d = s.Depth;
        v = d(isfinite(d) & d > 0);
        if ~isempty(v)
            lo = prctile(v, 2);
            hi = prctile(v, 98);
            imagesc(d, [lo hi]);
            axis image off;
            colormap(gca, turbo);
            cb = colorbar;
            cb.Label.String = 'Depth (m)';
            title(sprintf('Depth | Range: [%.2f, %.2f] m | FPS: %.1f', ...
                lo, hi, s.Metrics.DepthFps));
        else
            axis off;
            text(0.1, 0.5, 'Depth invalid');
        end
    else
        axis off;
        text(0.1, 0.5, 'No Depth');
    end

    drawnow;
end

zed.rDisconnect();

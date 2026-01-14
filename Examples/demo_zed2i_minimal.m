% demo_zed2i_minimal.m
% Minimal test for ZED2i wrapper.

clearvars
clear all
close all
clc

zed = ZED2i();
zed.rConnect();

hFig = figure('Name', 'ZED2i Minimal Preview', 'NumberTitle', 'off');

t0 = tic;
seconds = 10;

while ishandle(hFig) && toc(t0) < seconds
    ok = zed.rGrab();
    if ok
        img = zed.rGetImage();
        imshow(img);
        title(sprintf('FPS: %.1f | Drops: %d', ...
            zed.pData.Metrics.ImageFps, ...
            zed.pData.Metrics.ImageDrops));
        drawnow;
    else
        % Optional: print last error occasionally
        % disp(zed.pFlag.LastError);
        pause(0.01);
    end
end

zed.rDisconnect();

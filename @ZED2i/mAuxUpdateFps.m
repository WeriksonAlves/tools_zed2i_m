function mAuxUpdateFps(zed, stream)
%mAuxUpdateFps Update FPS estimate for the given stream ("image" or "depth").
%
%   zed.mAuxUpdateFps("image");
%   zed.mAuxUpdateFps("depth");

    alpha = zed.pPar.fpsAlpha;

    switch string(stream)
        case "image"
            if isempty(zed.pData.LastGrabTicImage)
                zed.pData.LastGrabTicImage = tic;
                return;
            end

            dt = toc(zed.pData.LastGrabTicImage);
            zed.pData.LastGrabTicImage = tic;

            if dt > 0
                fps  = 1 / dt;
                prev = zed.pData.Metrics.ImageFps;
                if prev <= 0
                    zed.pData.Metrics.ImageFps = fps;
                else
                    zed.pData.Metrics.ImageFps = (1 - alpha) * prev + alpha * fps;
                end
            end

        case "depth"
            if isempty(zed.pData.LastGrabTicDepth)
                zed.pData.LastGrabTicDepth = tic;
                return;
            end

            dt = toc(zed.pData.LastGrabTicDepth);
            zed.pData.LastGrabTicDepth = tic;

            if dt > 0
                fps  = 1 / dt;
                prev = zed.pData.Metrics.DepthFps;
                if prev <= 0
                    zed.pData.Metrics.DepthFps = fps;
                else
                    zed.pData.Metrics.DepthFps = (1 - alpha) * prev + alpha * fps;
                end
            end
    end
end

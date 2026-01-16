function ok = rGrab(zed)
%rGrab Blocking grab: receive image and depth messages and decode into MATLAB arrays.
%
% ok = true when at least one stream is successfully received and decoded.

    ok = false;

    if ~zed.pFlag.Connected
        zed.pFlag.LastError = 'Not connected. Call rConnect() first.';
        return;
    end

    % Grab image and depth independently, so a failure in one stream
    % does not prevent the other from being updated.
    okImage = grabImageStream(zed);
    okDepth = grabDepthStream(zed);

    ok = okImage || okDepth;
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function okImage = grabImageStream(zed)
%grabImageStream Receive and decode RGB image stream.

    okImage = false;

    try
        msgImg = receive(zed.pCom.subImage, zed.pPar.timeoutSec);
        zed.pCom.lastMsgImage = msgImg;

        zed.pData.Image = rosReadImage(msgImg);

        zed.pFlag.HasImage = true;
        okImage = true;

        updateFps(zed, 'image');

    catch excp
        zed.pData.Metrics.ImageDrops = zed.pData.Metrics.ImageDrops + 1;
        zed.pFlag.HasImage = false;
        zed.pFlag.LastError = ['Image grab failed: ' excp.message];
    end
end

function okDepth = grabDepthStream(zed)
%grabDepthStream Receive and decode depth stream with sanitization.

    okDepth = false;

    try
        msgDepth = receive(zed.pCom.subDepth, zed.pPar.timeoutSec);
        zed.pCom.lastMsgDepth = msgDepth;

        depth = rosReadImage(msgDepth);

        % --- Depth sanitization (safe default) ---
        depth(~isfinite(depth)) = NaN;   % keep invalid as NaN
        depth(depth <= 0) = NaN;         % non-positive is invalid for depth

        % Optional sanity: keep as single to reduce memory footprint
        if ~isa(depth, 'single')
            depth = single(depth);
        end

        zed.pData.Depth = depth;
        zed.pFlag.HasDepth = true;
        okDepth = true;

        updateFps(zed, 'depth');

    catch excp
        zed.pData.Metrics.DepthDrops = zed.pData.Metrics.DepthDrops + 1;
        zed.pFlag.HasDepth = false;
        zed.pFlag.LastError = ['Depth grab failed: ' excp.message];
    end
end

function updateFps(zed, stream)
%updateFps Update FPS estimate for the given stream ('image' or 'depth').

    alpha = 0.2;  % EMA smoothing factor

    switch stream
        case 'image'
            if isempty(zed.pData.LastGrabTicImage)
                zed.pData.LastGrabTicImage = tic;
                return;
            end

            dt = toc(zed.pData.LastGrabTicImage);
            zed.pData.LastGrabTicImage = tic;

            if dt > 0
                fps = 1 / dt;
                prev = zed.pData.Metrics.ImageFps;
                if prev <= 0
                    zed.pData.Metrics.ImageFps = fps;
                else
                    zed.pData.Metrics.ImageFps = (1 - alpha) * prev + alpha * fps;
                end
            end

        case 'depth'
            if isempty(zed.pData.LastGrabTicDepth)
                zed.pData.LastGrabTicDepth = tic;
                return;
            end

            dt = toc(zed.pData.LastGrabTicDepth);
            zed.pData.LastGrabTicDepth = tic;

            if dt > 0
                fps = 1 / dt;
                prev = zed.pData.Metrics.DepthFps;
                if prev <= 0
                    zed.pData.Metrics.DepthFps = fps;
                else
                    zed.pData.Metrics.DepthFps = (1 - alpha) * prev + alpha * fps;
                end
            end
    end
end

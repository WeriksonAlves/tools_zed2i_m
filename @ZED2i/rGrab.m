function ok = rGrab(zed)
%rGrab Blocking grab: receive image message and decode into MATLAB array.
%
% ok = true when a frame is successfully received and decoded.

    ok = false;

    if ~zed.pFlag.Connected
        zed.pFlag.LastError = 'Not connected. Call rConnect() first.';
        return;
    end

    try
        msg = receive(zed.pCom.subImage, zed.pPar.timeoutSec);
        zed.pCom.lastMsgImage = msg;

        % Decode image
        zed.pData.Image = rosReadImage(msg);
        zed.pFlag.HasImage = true;
        ok = true;

        % Update simple FPS estimate
        if isempty(zed.pData.LastGrabTic)
            zed.pData.LastGrabTic = tic;
        else
            dt = toc(zed.pData.LastGrabTic);
            zed.pData.LastGrabTic = tic;

            if dt > 0
                fps = 1 / dt;

                % Light smoothing (EMA)
                alpha = 0.2;
                prev = zed.pData.Metrics.ImageFps;
                if prev <= 0
                    zed.pData.Metrics.ImageFps = fps;
                else
                    zed.pData.Metrics.ImageFps = (1 - alpha) * prev + alpha * fps;
                end
            end
        end

    catch excp
        zed.pData.Metrics.ImageDrops = zed.pData.Metrics.ImageDrops + 1;
        zed.pFlag.HasImage = false;
        zed.pFlag.LastError = excp.message;
    end
end

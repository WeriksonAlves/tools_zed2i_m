function iControlVariables(zed)
%iControlVariables Initialize flags, buffers, and minimal metrics.

    % Flags
    zed.pFlag.Connected = false;
    zed.pFlag.HasImage = false;
    zed.pFlag.LastError = '';

    % Data buffers
    zed.pData.Image = [];
    zed.pData.LastGrabTic = [];  % tic handle (for simple FPS estimate)

    % Minimal metrics
    zed.pData.Metrics = struct();
    zed.pData.Metrics.ImageFps = 0;
    zed.pData.Metrics.ImageDrops = 0;

    % ROS handles / last message
    zed.pCom = struct();
    zed.pCom.node = [];
    zed.pCom.subImage = [];
    zed.pCom.lastMsgImage = [];
end

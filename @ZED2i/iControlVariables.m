function iControlVariables(zed)
%iControlVariables Initialize flags, buffers, and minimal metrics.

    % Flags
    zed.pFlag.Connected = false;
    zed.pFlag.HasImage = false;
    zed.pFlag.HasDepth = false;          % NEW
    zed.pFlag.LastError = '';

    % Data buffers
    zed.pData.Image = [];
    zed.pData.Depth = [];                % NEW
    zed.pData.LastGrabTicImage = [];      % NEW (separado por stream)
    zed.pData.LastGrabTicDepth = [];      % NEW

    % Minimal metrics
    zed.pData.Metrics = struct();
    zed.pData.Metrics.ImageFps = 0;
    zed.pData.Metrics.ImageDrops = 0;
    zed.pData.Metrics.DepthFps = 0;       % NEW
    zed.pData.Metrics.DepthDrops = 0;     % NEW

    % ROS handles / last message
    zed.pCom = struct();
    zed.pCom.node = [];
    zed.pCom.subImage = [];
    zed.pCom.subDepth = [];              % NEW
    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];          % NEW
end

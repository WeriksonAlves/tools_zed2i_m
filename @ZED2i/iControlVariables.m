function iControlVariables(zed)
%iControlVariables Initialize flags, buffers, and minimal metrics.

    % Flags
    zed.pFlag.Connected = false;
    zed.pFlag.HasImage = false;
    zed.pFlag.HasDepth = false;
    zed.pFlag.LastError = '';
    zed.pFlag.HasCalibration = false;

    % Data buffers
    zed.pData.Image = [];
    zed.pData.Depth = [];
    zed.pData.LastGrabTicImage = [];
    zed.pData.LastGrabTicDepth = [];
    zed.pData.Calibration = struct();

    % Minimal metrics
    zed.pData.Metrics = struct();
    zed.pData.Metrics.ImageFps = 0;
    zed.pData.Metrics.ImageDrops = 0;
    zed.pData.Metrics.DepthFps = 0;
    zed.pData.Metrics.DepthDrops = 0;

    % ROS handles / last message
    zed.pCom = struct();
    zed.pCom.node = [];
    zed.pCom.subImage = [];
    zed.pCom.subDepth = [];
    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];
    zed.pCom.subInfo = [];
    zed.pCom.lastMsgInfo = [];

end

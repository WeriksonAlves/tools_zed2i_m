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

    % -------------------- IMU (optional) --------------------
    zed.pFlag.HasImu = false;

    zed.pData.Imu = struct();  % last decoded IMU snapshot

    zed.pCom.subImu = [];
    zed.pCom.lastMsgImu = [];

    % -------------------- Pose/Odom (optional) --------------------
    zed.pFlag.HasPose = false;

    zed.pData.Pose = struct();  % last decoded pose snapshot

    zed.pCom.subOdom = [];
    zed.pCom.lastMsgOdom = [];

    % -------------------- PointCloud (optional) --------------------
    zed.pFlag.HasPointCloud = false;

    zed.pData.PointCloud = struct();  % last decoded point cloud snapshot

    zed.pCom.subPointCloud     = [];
    zed.pCom.lastMsgPointCloud = [];


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

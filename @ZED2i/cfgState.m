function cfgState(zed)
%cfgState Initialize flags, buffers, and minimal metrics.
%
% Responsável apenas por colocar o objeto em um estado interno conhecido,
% sem tocar em ROS (sem criar nó ou subscribers).

    % ---------------------------------------------------------------------
    % Flags de estado
    % ---------------------------------------------------------------------
    zed.pFlag = struct();
    zed.pFlag.Connected      = false;
    zed.pFlag.HasImage       = false;
    zed.pFlag.HasDepth       = false;
    zed.pFlag.HasCalibration = false;
    zed.pFlag.HasImu         = false;
    zed.pFlag.HasPose        = false;
    zed.pFlag.HasPointCloud  = false;
    zed.pFlag.LastError      = "";

    % ---------------------------------------------------------------------
    % Buffers de dados + métricas
    % ---------------------------------------------------------------------
    zed.pData = struct();

    % Visual
    zed.pData.Image           = [];
    zed.pData.Depth           = [];
    zed.pData.LastGrabTicImage = [];
    zed.pData.LastGrabTicDepth = [];

    % Calibração
    zed.pData.Calibration = struct();

    % IMU / Pose / PointCloud (snapshots)
    zed.pData.Imu        = struct();
    zed.pData.Pose       = struct();
    zed.pData.PointCloud = struct();

    % Métricas mínimas
    zed.pData.Metrics = struct();
    zed.pData.Metrics.ImageFps   = 0;
    zed.pData.Metrics.DepthFps   = 0;
    zed.pData.Metrics.ImageDrops = 0;
    zed.pData.Metrics.DepthDrops = 0;

    % ---------------------------------------------------------------------
    % Comunicação ROS2 (node, subscribers, últimas mensagens)
    % Reuso da lógica centralizada em resetCommState
    % ---------------------------------------------------------------------
    zed.utilResetCommState();
end

function iState(zed)
%iState Initialize flags, buffers, and minimal metrics.
%
% Only sets internal state. No ROS calls are made here.

    % ---------------------------------------------------------------------
    % Flags
    % ---------------------------------------------------------------------
    pFlag = struct();
    pFlag.Connected      = false;
    pFlag.HasImage       = false;
    pFlag.HasDepth       = false;
    pFlag.HasCalibration = false;
    pFlag.HasImu         = false;
    pFlag.HasPose        = false;
    pFlag.HasPointCloud  = false;
    pFlag.LastError      = "";

    % ---------------------------------------------------------------------
    % Data buffers
    % ---------------------------------------------------------------------
    pData = struct();

    pData.Image            = [];
    pData.Depth            = [];
    pData.DepthMask        = [];
    pData.LastGrabTicImage = [];
    pData.LastGrabTicDepth = [];

    pData.Calibration = zed.sEmptyCalibration();
    pData.Intrinsics = [];

pData.Imu        = struct();
    pData.Pose       = struct();
    pData.PointCloud = struct();

    pData.Metrics = defaultMetricsStruct_();

    % ---------------------------------------------------------------------
    % CAD state (lazy-load ready)
    % ---------------------------------------------------------------------
    pCAD = struct();
    pCAD.obj = {};
    pCAD.mtl = {};
    pCAD.parts = struct([]);
    pCAD.flagLoaded  = 0;
    pCAD.flagCreated = 0;
    pCAD.flagLines   = 0;

    % ---------------------------------------------------------------------
    % Assign once (reduces handle writes)
    % ---------------------------------------------------------------------
    zed.pFlag = pFlag;
    zed.pData = pData;
    
    % --- CAD state (always exists; filled by mCADload when used)
    zed.pCAD = struct( ...
        "Loaded", false, ...
        "Handle", [], ...
        "Faces", [], ...
        "Vertices", [], ...
        "Colors", [], ...
        "ObjPath", "", ...
        "MtlPath", "" ...
    );


    % ---------------------------------------------------------------------
    % ROS2 communication state
    % ---------------------------------------------------------------------
    zed.sResetCommState();

    % ---------------------------------------------------------------------
    % Stream throttling state
    % ---------------------------------------------------------------------
    if ~isfield(zed.pData, "Stream") || ~isstruct(zed.pData.Stream)
        zed.pData.Stream = struct();
    end

    % lastUpdateSec: last time (in seconds) we committed each stream
    zed.pData.Stream.lastUpdateSec = struct( ...
        "image", -Inf, ...
        "depth", -Inf, ...
        "imu",   -Inf, ...
        "pose",  -Inf, ...
        "pcd",   -Inf, ...
        "calib", -Inf ...
    );
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------
function cal = defaultCalibrationStruct_()
    cal = struct();
    cal.FocalLength          = [];
    cal.PrincipalPoint       = [];
    cal.ImageSize            = [];
    cal.RadialDistortion     = [];
    cal.TangentialDistortion = [];
    cal.Skew                 = 0;
    cal.K                    = [];
    cal.D                    = [];
end

function metrics = defaultMetricsStruct_()
    metrics = struct();
    metrics.ImageFps   = 0;
    metrics.DepthFps   = 0;
    metrics.ImageDrops = 0;
    metrics.DepthDrops = 0;
end

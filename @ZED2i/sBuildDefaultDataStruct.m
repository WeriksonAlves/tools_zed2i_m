function data = sBuildDefaultDataStruct(zed)
%sBuildDefaultDataStruct Deterministic output contract for getSensorData.
%
% NOTE: Keep this struct stable; user scripts may rely on these fields.

    data = struct();

    data.TimestampSec = 0.0;
    data.Connected = false;

    data.HasImage = false;
    data.HasDepth = false;
    data.HasImu = false;
    data.HasPose = false;
    data.HasPointCloud = false;
    data.HasCalibration = false;

    data.Image = [];
    data.Depth = [];
    data.DepthMask = [];
    data.Imu = zed.sEmptyImu();
    data.Pose = zed.sEmptyPose();
    data.PointCloud = zed.sEmptyPointCloud();

    % Model 1: Calibration is a single struct including intrinsics.
    data.Calibration = zed.sEmptyCalibration();

    % Backward-compatible convenience mirror (may be empty).
    data.Intrinsics = data.Calibration.Intrinsics;

    data.Metrics = zed.sSafeGetMetrics();
    data.LastError = zed.sSafeLastError();
end

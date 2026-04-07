function rosDisconnect(zed)
%rosDisconnect Release ROS2 resources and reset communication state.

    zed.sResetCommState();

    zed.pFlag.Connected      = false;
    zed.pFlag.HasImage       = false;
    zed.pFlag.HasDepth       = false;
    zed.pFlag.HasCalibration = false;

    % Optional flags: set if they exist (safe for older objects)
    if isfield(zed.pFlag, "HasImu");        zed.pFlag.HasImu = false; end
    if isfield(zed.pFlag, "HasPose");       zed.pFlag.HasPose = false; end
    if isfield(zed.pFlag, "HasPointCloud"); zed.pFlag.HasPointCloud = false; end
end

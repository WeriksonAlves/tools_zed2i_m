function rosDisconnect(zed)
%rosDisconnect Release ROS2 resources and reset communication state.
%
% This method explicitly releases ROS2 subscribers, cached messages, and
% connection flags. It is safe to call multiple times.

    % ---------------------------------------------------------------------
    % Release ROS2 communication resources
    % ---------------------------------------------------------------------
    zed.sResetCommState();

    % ---------------------------------------------------------------------
    % Reset connection flags
    % ---------------------------------------------------------------------
    if isempty(zed.pFlag) || ~isstruct(zed.pFlag)
        zed.pFlag = struct();
    end

    zed.pFlag.Connected      = false;
    zed.pFlag.HasImage       = false;
    zed.pFlag.HasDepth       = false;
    zed.pFlag.HasCalibration = false;
    zed.pFlag.LastError      = "";

    % Optional flags: set if they exist or create them for stable shape.
    zed.pFlag.HasImu        = false;
    zed.pFlag.HasPose       = false;
    zed.pFlag.HasPointCloud = false;

    % ---------------------------------------------------------------------
    % Optional: reset cached data buffers.
    %
    % This avoids stale data being interpreted as valid after disconnect.
    % ---------------------------------------------------------------------
    if isstruct(zed.pData)
        if isfield(zed.pData, "Image")
            zed.pData.Image = [];
        end
        if isfield(zed.pData, "Depth")
            zed.pData.Depth = [];
        end
        if isfield(zed.pData, "DepthMask")
            zed.pData.DepthMask = [];
        end
        if isfield(zed.pData, "Calibration")
            zed.pData.Calibration = zed.sEmptyCalibration();
        end
        if isfield(zed.pData, "Intrinsics")
            zed.pData.Intrinsics = [];
        end
        if isfield(zed.pData, "Imu")
            zed.pData.Imu = zed.sEmptyImu();
        end
        if isfield(zed.pData, "Pose")
            zed.pData.Pose = zed.sEmptyPose();
        end
        if isfield(zed.pData, "PointCloud")
            zed.pData.PointCloud = zed.sEmptyPointCloud();
        end
    end
end
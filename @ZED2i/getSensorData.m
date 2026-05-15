function data = getSensorData(zed)
%getSensorData Scheduled acquisition + SINGLE commit point for ZED2i class state.
%
% Model 1 (clean):
%   - rosGet* methods are decode-only (no writes to zed.pData / zed.pFlag).
%   - getSensorData is the ONLY place that commits decoded data into:
%       zed.pData / zed.pPos / zed.pFlag
%
% Streams (names must match pPar.streamHz fields):
%   image, depth, imu, pose, pcd, calib

    % ---------------------------------------------------------------------
    % Default output (no I/O)
    % ---------------------------------------------------------------------
    data = zed.sBuildDefaultDataStruct();
    data.TimestampSec = zed.sNowSec();
    data.Connected = zed.sSafeFlag("Connected");

    % Not connected -> no I/O
    if ~data.Connected
        data.LastError = "Not connected. Call rosConnect() first.";
        return;
    end

    errList = strings(0);

    % Decide decode mode:
    % - callbacks enabled -> consume cached messages (non-blocking)
    % - callbacks disabled -> fetch via receive() (blocking up to timeoutSec)
    mode = "cached";
    if isfield(zed.pPar, "UseCallbacks") && ~logical(zed.pPar.UseCallbacks)
        mode = "fetch";
    end

    % ---------------------------------------------------------------------
    % CALIBRATION (CameraInfo) [lightweight]
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("calib")
        [calib, intr, ok, err] = zed.rosGetCalibration("Mode", "fetch");
        if ok
            % Model 1: single struct includes intrinsics
            calib.Intrinsics = intr;

            zed.pData.Calibration = calib;
            zed.pData.Intrinsics = intr; % backward-compatible mirror

            zed.pFlag.HasCalibration = true;
        else
            if strlength(err) > 0
                errList(end+1) = "Calibration: " + err; %#ok<AGROW>
            end
        end
        zed.sMarkFetch("calib");
    end

    % ---------------------------------------------------------------------
    % IMAGE
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("image")
        [img, ok, err] = zed.rosGetImage("Mode", mode);

        if ~ok && mode == "cached"
            [img, ok, err] = zed.rosGetImage("Mode", "fetch");
        end

        if ok
            zed.pData.Image = img;
            zed.pFlag.HasImage = true;
            zed.sUpdateFps("image");
        else
            if strlength(err) > 0
                errList(end+1) = "Image: " + err; %#ok<AGROW>
            end
        end

        zed.sMarkFetch("image");
    end

    % ---------------------------------------------------------------------
    % DEPTH
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("depth")
        [depth, depthMask, ok, err] = zed.rosGetDepth("Mode", mode);

        if ~ok && mode == "cached"
            [depth, depthMask, ok, err] = zed.rosGetDepth("Mode", "fetch");
        end

        if ok
            zed.pData.Depth = depth;
            zed.pData.DepthMask = depthMask;
            zed.pFlag.HasDepth = true;
            zed.sUpdateFps("depth");
        else
            if strlength(err) > 0
                errList(end+1) = "Depth: " + err; %#ok<AGROW>
            end
        end

        zed.sMarkFetch("depth");
    end

    % ---------------------------------------------------------------------
    % IMU
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("imu") && isfield(zed.pPar, "enableImu") && logical(zed.pPar.enableImu)
    [imu, ok, err] = zed.rosGetImu("Mode", mode);

        if ~ok && mode == "cached"
            [imu, ok, err] = zed.rosGetImu("Mode", "fetch");
        end

        if ok
            zed.pData.Imu = imu;
            zed.pData.Imu.TimestampSec = data.TimestampSec;
            zed.pFlag.HasImu = true;
        else
            if strlength(err) > 0
                errList(end+1) = "Imu: " + err; %#ok<AGROW>
            end
        end
        zed.sMarkFetch("imu");
    end

    % ---------------------------------------------------------------------
    % POSE
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("pose") && isfield(zed.pPar, "enablePose") && logical(zed.pPar.enablePose)
        [pose, ok, err] = zed.rosGetPose("Mode", mode);

        if ~ok && mode == "cached"
            [pose, ok, err] = zed.rosGetPose("Mode", "fetch");
        end

        if ok
            zed.pData.Pose = pose;
            zed.pData.Pose.TimestampSec = data.TimestampSec;
            zed.pFlag.HasPose = true;

            % Commit to pPos convention.
            if isstruct(pose) && isfield(pose, "Position") && isfield(pose, "OrientationEuler")
                if ~isfield(zed.pPos, "X") || numel(zed.pPos.X) < 12
                    zed.pPos.X = zeros(12, 1);
                end
                zed.pPos.X(1:3) = pose.Position(:);
                zed.pPos.X(4:6) = pose.OrientationEuler(:);
            end
        else
            if strlength(err) > 0
                errList(end+1) = "Pose: " + err; %#ok<AGROW>
            end
        end
        zed.sMarkFetch("pose");
    end

    % ---------------------------------------------------------------------
    % POINT CLOUD (heavy)
    % ---------------------------------------------------------------------
    if zed.sShouldFetch("pcd") && isfield(zed.pPar, "enablePointCloud") && logical(zed.pPar.enablePointCloud)
        [pc, ok, err] = zed.rosGetPointCloud("Mode", mode);

        if ~ok && mode == "cached"
            [pc, ok, err] = zed.rosGetPointCloud("Mode", "fetch");
        end

        if ok
            zed.pData.PointCloud = pc;
            zed.pData.PointCloud.TimestampSec = data.TimestampSec;
            zed.pFlag.HasPointCloud = true;
        else
            if strlength(err) > 0
                errList(end+1) = "PointCloud: " + err; %#ok<AGROW>
            end
        end
        zed.sMarkFetch("pcd");
    end

    % ---------------------------------------------------------------------
    % Snapshot committed state -> output struct (single source of truth)
    % ---------------------------------------------------------------------
    data.HasImage = logical(zed.pFlag.HasImage);
    data.HasDepth = logical(zed.pFlag.HasDepth);
    data.HasImu = logical(zed.pFlag.HasImu);
    data.HasPose = logical(zed.pFlag.HasPose);
    data.HasPointCloud = logical(zed.pFlag.HasPointCloud);
    data.HasCalibration = logical(zed.pFlag.HasCalibration);

    if isfield(zed.pData, "Image")
        data.Image = zed.pData.Image;
    end
    if isfield(zed.pData, "Depth")
        data.Depth = zed.pData.Depth;
    end
    if isfield(zed.pData, "DepthMask")
        data.DepthMask = zed.pData.DepthMask;
    end
    if isfield(zed.pData, "Imu")
        data.Imu = zed.pData.Imu;
    end
    if isfield(zed.pData, "Pose")
        data.Pose = zed.pData.Pose;
    end
    if isfield(zed.pData, "PointCloud")
        data.PointCloud = zed.pData.PointCloud;
    end
    if isfield(zed.pData, "Calibration")
        data.Calibration = zed.pData.Calibration;
        if isfield(zed.pData.Calibration, "Intrinsics")
            data.Intrinsics = zed.pData.Calibration.Intrinsics;
        else
            data.Intrinsics = zed.pData.Intrinsics;
        end
    end

    data.Metrics = zed.sSafeGetMetrics();

    % Store only errors generated in the current acquisition cycle.
    % Do not merge the previous LastError, otherwise the error string grows
    % indefinitely across calls.
    if ~isempty(errList)
        zed.pFlag.LastError = strjoin(unique(errList, "stable"), " | ");
    else
        zed.pFlag.LastError = "";
    end

    data.LastError = zed.pFlag.LastError;
end

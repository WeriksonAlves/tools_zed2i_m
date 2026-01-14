function data = rGetSensorData(zed)
%rGetSensorData Aggregate sensor data in a lab-friendly format.
%
% Returns a struct with:
%   - Image (RGB)
%   - Depth (meters, single)
%   - Calibration (struct with K, D, etc.)
%   - Flags: Connected/HasImage/HasDepth/HasCalibration
%   - Metrics: FPS/Drops
%   - Timestamp: local MATLAB time (datetime)
%   - LastError

    data = struct();

    % -------------------- Basic flags --------------------
    data.Timestamp = datetime('now');
    data.Connected = logical(zed.pFlag.Connected);

    if ~zed.pFlag.Connected
        data.HasImage = false;
        data.HasDepth = false;
        data.HasCalibration = logical(isfield(zed.pFlag, 'HasCalibration') && zed.pFlag.HasCalibration);

        data.Image = [];
        data.Depth = [];
        data.Calibration = struct();

        data.Metrics = safeGetMetrics(zed);
        data.LastError = 'Not connected. Call rConnect() first.';
        return;
    end

    % -------------------- Grab streams --------------------
    ok = zed.rGrab(); %#ok<NASGU>

    data.HasImage = logical(isfield(zed.pFlag, 'HasImage') && zed.pFlag.HasImage);
    data.HasDepth = logical(isfield(zed.pFlag, 'HasDepth') && zed.pFlag.HasDepth);

    data.Image = zed.rGetImage();
    data.Depth = zed.rGetDepth();

    % -------------------- Calibration (lazy) --------------------
    if ~isfield(zed.pFlag, 'HasCalibration') || ~zed.pFlag.HasCalibration
        % Try to fetch once (uses cache if available)
        calib = zed.rGetCalibration();
        if ~isempty(fieldnames(calib))
            data.Calibration = calib;
        else
            data.Calibration = struct();
        end
    else
        data.Calibration = zed.pData.Calibration;
    end
    data.HasCalibration = logical(isfield(zed.pFlag, 'HasCalibration') && zed.pFlag.HasCalibration);

    % -------------------- Metrics & errors --------------------
    data.Metrics = safeGetMetrics(zed);

    if isfield(zed.pFlag, 'LastError')
        data.LastError = zed.pFlag.LastError;
    else
        data.LastError = '';
    end
end

function metrics = safeGetMetrics(zed)
    metrics = struct();

    if isfield(zed, 'pData') && isfield(zed.pData, 'Metrics')
        metrics = zed.pData.Metrics;
        return;
    end

    % Fallback defaults if Metrics is missing
    metrics.ImageFps = 0;
    metrics.DepthFps = 0;
    metrics.ImageDrops = 0;
    metrics.DepthDrops = 0;
end

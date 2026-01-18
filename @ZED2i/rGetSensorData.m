function data = rGetSensorData(zed)
%rGetSensorData Aggregate sensor data in a lab-friendly format.
%
% Returns a struct with stable fields:
%   Timestamp (datetime)
%   Connected (logical)
%   HasImage, HasDepth, HasCalibration (logical)
%   Image, Depth
%   Calibration (struct)
%   Metrics (struct)
%   LastError (char/string)

    data = buildDefaultDataStruct(zed);

    if ~zed.pFlag.Connected
        data.LastError = 'Not connected. Call rConnect() first.';
        return;
    end

    % Grab streams (updates internal buffers and flags)
    zed.rGrab();

    % Copy buffers (lab-friendly snapshot)
    data.HasImage = safeFlag(zed.pFlag, 'HasImage');
    data.HasDepth = safeFlag(zed.pFlag, 'HasDepth');

    data.Image = zed.rGetImage();
    data.Depth = zed.rGetDepth();

    % Optional IMU snapshot (lazy)
    if isfield(zed.pPar, 'enableImu') && zed.pPar.enableImu
        imu = zed.rGetImu();
        data.Imu = imu;
        data.HasImu = safeFlag(zed.pFlag, 'HasImu');
    else
        data.Imu = struct();
        data.HasImu = false;
    end


    % Lazy calibration (use cached if available; otherwise attempt once)
    [data.Calibration, data.HasCalibration] = getCalibrationSnapshot(zed);

    % Metrics + last error
    data.Metrics = safeGetMetrics(zed);
    data.LastError = safeLastError(zed);
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function data = buildDefaultDataStruct(zed)
%buildDefaultDataStruct Create a deterministic output struct shape.

    data = struct();
    data.Timestamp = datetime('now');

    data.Connected = safeFlag(zed.pFlag, 'Connected');
    data.HasImage = false;
    data.HasDepth = false;
    data.HasCalibration = safeFlag(zed.pFlag, 'HasCalibration');
    data.HasImu = safeFlag(zed.pFlag, 'HasImu');
    data.Imu = zed.rGetImu();

    data.Image = [];
    data.Depth = [];
    data.Calibration = struct();
    data.Metrics = safeGetMetrics(zed);
    data.LastError = safeLastError(zed);
end

function value = safeFlag(flags, fieldName)
%safeFlag Return a logical flag field if available, otherwise false.

    value = false;

    if isstruct(flags) && isfield(flags, fieldName)
        value = logical(flags.(fieldName));
    end
end

function err = safeLastError(zed)
%safeLastError Return the last error string if available.

    err = '';

    if isfield(zed, 'pFlag') && isstruct(zed.pFlag) && isfield(zed.pFlag, 'LastError')
        err = zed.pFlag.LastError;
    end
end

function [calib, hasCalib] = getCalibrationSnapshot(zed)
%getCalibrationSnapshot Return calibration struct and flag.
%
% Prefers cached calibration if available. Otherwise attempts a single fetch.
% Never throws: returns empty struct on failure.

    calib = struct();
    hasCalib = safeFlag(zed.pFlag, 'HasCalibration');

    if hasCalib && isfield(zed.pData, 'Calibration') && ~isempty(fieldnames(zed.pData.Calibration))
        calib = zed.pData.Calibration;
        return;
    end

    % Attempt once (rGetCalibration already caches on success)
    try
        calibTry = zed.rGetCalibration();
        if ~isempty(fieldnames(calibTry))
            calib = calibTry;
            hasCalib = true;
        end
    catch
        % Keep defaults on failure (no throw)
        hasCalib = false;
        calib = struct();
    end
end

function metrics = safeGetMetrics(zed)
%safeGetMetrics Return metrics struct with stable fields.

    metrics = struct( ...
        'ImageFps', 0, ...
        'DepthFps', 0, ...
        'ImageDrops', 0, ...
        'DepthDrops', 0 ...
    );

    if isfield(zed, 'pData') && isstruct(zed.pData) && isfield(zed.pData, 'Metrics')
        src = zed.pData.Metrics;

        % Copy only known fields (stable output contract)
        metrics = copyIfField(metrics, src, 'ImageFps');
        metrics = copyIfField(metrics, src, 'DepthFps');
        metrics = copyIfField(metrics, src, 'ImageDrops');
        metrics = copyIfField(metrics, src, 'DepthDrops');
    end
end

function dst = copyIfField(dst, src, fieldName)
%copyIfField Copy a field from src to dst if it exists.

    if isstruct(src) && isfield(src, fieldName)
        dst.(fieldName) = src.(fieldName);
    end
end

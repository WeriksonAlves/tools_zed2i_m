function data = rGetSensorData(zed)
%rGetSensorData Aggregate sensor data in a lab-friendly format.
%
% Returns a struct with stable fields:
%   Timestamp            (datetime)
%   Connected            (logical)
%   HasImage, HasDepth   (logical)
%   HasImu, HasPose      (logical)
%   HasPointCloud        (logical)
%   Image                (RGB frame or [])
%   Depth                (depth frame or [])
%   DepthMask            (logical mask of valid depth or [])
%   Imu                  (struct)
%   Pose                 (struct)
%   PointCloud           (struct)
%   Metrics              (struct with FPS/drops)
%   LastError            (char/string)

    % ---------------------------------------------------------------------
    % Default shape (sem chamadas de I/O)
    % ---------------------------------------------------------------------
    data = buildDefaultDataStruct(zed);

    % ---------------------------------------------------------------------
    % Conectado?
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        data.LastError = "Not connected. Call lcConnect() first.";
        return;
    end

    % ---------------------------------------------------------------------
    % Visual: image + depth (cada getter faz seu próprio receive)
    % ---------------------------------------------------------------------
    img = zed.rGetImage();
    [depth, depthMask] = zed.rGetDepth();

    data.Image     = img;
    data.Depth     = depth;
    data.DepthMask = depthMask;

    data.HasImage = ~isempty(img)  && safeFlag(zed.pFlag, "HasImage");
    data.HasDepth = ~isempty(depth) && safeFlag(zed.pFlag, "HasDepth");

    % ---------------------------------------------------------------------
    % IMU (opcional, só se habilitado)
    % ---------------------------------------------------------------------
    if isfield(zed.pPar, "enableImu") && zed.pPar.enableImu
        imu = zed.rGetImu();
        data.Imu    = imu;
        data.HasImu = safeFlag(zed.pFlag, "HasImu");
    end

    % ---------------------------------------------------------------------
    % Pose (opcional, só se habilitado)
    % ---------------------------------------------------------------------
    if isfield(zed.pPar, "enablePose") && zed.pPar.enablePose
        p = zed.rGetPose();
        data.Pose    = p;
        data.HasPose = safeFlag(zed.pFlag, "HasPose");
    end

    % ---------------------------------------------------------------------
    % PointCloud (sem auto-fetch; usa último valor decodificado, se houver)
    % ---------------------------------------------------------------------
    if isfield(zed.pPar, "enablePointCloud") && zed.pPar.enablePointCloud
        % Variante: heavy auto-fetch (explicity to config)
        if isfield(zed.pPar, "autoFetchPointCloud") && zed.pPar.autoFetchPointCloud
            try
                % This may be a heavy operation
                pc = zed.rGetPointCloud();
            catch
                % Ignore errors here; use last known point cloud
            end
        end

        if isfield(zed.pData, "PointCloud") && safeFlag(zed.pFlag, "HasPointCloud")
            data.PointCloud    = zed.pData.PointCloud;
            data.HasPointCloud = true;
        end
    end

    % ---------------------------------------------------------------------
    % Metrics + last error (snapshot)
    % ---------------------------------------------------------------------
    data.Metrics   = safeGetMetrics(zed);
    data.LastError = safeLastError(zed);
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function data = buildDefaultDataStruct(zed)
%buildDefaultDataStruct Create a deterministic output struct shape.

    data = struct();

    data.Timestamp = datetime("now");
    data.Connected = safeFlag(zed.pFlag, "Connected");

    % Flags
    data.HasImage       = false;
    data.HasDepth       = false;
    data.HasPointCloud  = false;
    data.HasImu         = safeFlag(zed.pFlag, "HasImu");
    data.HasPose        = safeFlag(zed.pFlag, "HasPose");
    data.HasPointCloud  = safeFlag(zed.pFlag, "HasPointCloud");

    % Visual
    data.Image     = [];
    data.Depth     = [];
    data.DepthMask = [];

    % Outros sensores
    data.Imu        = struct();
    data.Pose       = struct();
    data.PointCloud = struct();

    % Métricas + erro
    data.Metrics   = safeGetMetrics(zed);
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

    err = "";

    if isstruct(zed.pFlag) && isfield(zed.pFlag, "LastError")
        err = zed.pFlag.LastError;
    end
end

function metrics = safeGetMetrics(zed)
%safeGetMetrics Return metrics struct with stable fields.

    metrics = struct( ...
        "ImageFps",   0, ...
        "DepthFps",   0, ...
        "ImageDrops", 0, ...
        "DepthDrops", 0 ...
    );
    if isstruct(zed.pData) && ...
       isfield(zed.pData, "Metrics") && ...
       isstruct(zed.pData.Metrics)

        src = zed.pData.Metrics;

        % Copy only known fields (stable output contract)
        metrics = copyIfField(metrics, src, "ImageFps");
        metrics = copyIfField(metrics, src, "DepthFps");
        metrics = copyIfField(metrics, src, "ImageDrops");
        metrics = copyIfField(metrics, src, "DepthDrops");
    end
end

function dst = copyIfField(dst, src, fieldName)
%copyIfField Copy a field from src to dst if it exists.

    if isstruct(src) && isfield(src, fieldName)
        dst.(fieldName) = src.(fieldName);
    end
end

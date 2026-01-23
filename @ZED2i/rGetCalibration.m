function [calib, intr] = rGetCalibration(zed)
%rGetCalibration Get camera intrinsics/extrinsics from ROS2 CameraInfo.
%
% Outputs:
%   calib : struct with fields
%       K (3x3), D (Nx1), R (3x3), P (3x4),
%       Width, Height, DistortionModel
%   intr  : cameraIntrinsics (Computer Vision Toolbox), or [] if unavailable

    calib = buildEmptyCalibrationStruct();
    intr  = [];

    % ------------------------------------------------------------
    % Fast path: use cached calibration if already available
    % ------------------------------------------------------------
    if safeFlag(zed.pFlag, "HasCalibration") && ...
       isfield(zed.pData, "Calibration") && ...
       isfield(zed.pData.Calibration, "K") && ...
       ~isempty(zed.pData.Calibration.K)

        calib = zed.pData.Calibration;
        intr  = safeParseIntrinsics(calib);
        return;
    end

    % ------------------------------------------------------------
    % Safety check: require connection
    % ------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        return;
    end

    % ------------------------------------------------------------
    % Get CameraInfo message (prefer cached, otherwise receive)
    % ------------------------------------------------------------
    msg = [];

    if isfield(zed.pCom, "lastMsgInfo") && ~isempty(zed.pCom.lastMsgInfo)
        msg = zed.pCom.lastMsgInfo;
    end

    if isempty(msg)
        try
            msg = receive(zed.pCom.subInfo, zed.pPar.timeoutSec);
            zed.pCom.lastMsgInfo = msg;
        catch excp
            zed.pFlag.LastError = "CameraInfo receive failed: " + excp.message;
            return;
        end
    end

    % ------------------------------------------------------------
    % Parse CameraInfo -> calib
    % ------------------------------------------------------------
    try
        calib = parseCameraInfoMessage(msg);

        % Cache results
        zed.pData.Calibration = calib;
        zed.pFlag.HasCalibration = true;

    catch excp
        zed.pFlag.LastError = "CameraInfo parse failed: " + excp.message;
        calib = buildEmptyCalibrationStruct();
        intr  = [];
        return;
    end

    % ------------------------------------------------------------
    % Optionally build MATLAB cameraIntrinsics
    % ------------------------------------------------------------
    intr = safeParseIntrinsics(calib);
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function calib = buildEmptyCalibrationStruct()
%buildEmptyCalibrationStruct Create a calibration struct with known fields.

    calib = struct( ...
        "K", [], ...
        "D", [], ...
        "R", [], ...
        "P", [], ...
        "Width", [], ...
        "Height", [], ...
        "DistortionModel", "" ...
    );
end

function flag = safeFlag(flags, fieldName)
%safeFlag Return logical flag if available, otherwise false.

    flag = false;

    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

function calib = parseCameraInfoMessage(msg)
%parseCameraInfoMessage Convert ROS2 CameraInfo into a calibration struct.

    calib = buildEmptyCalibrationStruct();

    % CameraInfo fields:
    % msg.k (9), msg.d (N), msg.r (9), msg.p (12)
    K = reshape(double(msg.k), [3, 3])';
    R = reshape(double(msg.r), [3, 3])';
    P = reshape(double(msg.p), [3, 4])';

    calib.K = K;
    calib.D = double(msg.d(:));
    calib.R = R;
    calib.P = P;

    calib.Width  = double(msg.width);
    calib.Height = double(msg.height);

    % Em ROS2, distortion_model vem como char/string; normalizamos para char
    calib.DistortionModel = char(msg.distortion_model);
end

function intr = safeParseIntrinsics(calib)
%safeParseIntrinsics Build cameraIntrinsics if possible (optional dependency).
%
% Returns [] se:
%   - calib incompleto (sem K ou tamanho), ou
%   - Computer Vision Toolbox indisponível.

    intr = [];

    % Checagem rápida de consistência
    if ~isstruct(calib) || ~isfield(calib, "K") || isempty(calib.K) || ...
       ~isfield(calib, "Width") || isempty(calib.Width) || ...
       ~isfield(calib, "Height") || isempty(calib.Height)
        return;
    end

    try
        fx = calib.K(1, 1);
        fy = calib.K(2, 2);
        cx = calib.K(1, 3);
        cy = calib.K(2, 3);

        % MATLAB expects [fx fy] and principal point [cx cy]
        focalLength    = [fx fy];
        principalPoint = [cx cy];
        imageSize      = [calib.Height calib.Width]; % [rows cols]

        intr = cameraIntrinsics(focalLength, principalPoint, imageSize);
    catch
        % Falha provavelmente por ausência do CV Toolbox ou dados inválidos
        intr = [];
    end
end

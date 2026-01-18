function calib = rGetCalibration(zed)
%rGetCalibration Get camera intrinsics/extrinsics from ROS2 CameraInfo.
%
% Returns a struct with:
%   K (3x3), D (Nx1), R (3x3), P (3x4),
%   Width, Height, DistortionModel

    calib = buildEmptyCalibrationStruct();

    % ------------------------------------------------------------
    % Fast path: return cached calibration if already available
    % ------------------------------------------------------------
    if safeFlag(zed.pFlag, "HasCalibration") && ...
       isfield(zed.pData, "Calibration") && ...
       ~isempty(fieldnames(zed.pData.Calibration))
        calib = zed.pData.Calibration;
        return;
    end

    % ------------------------------------------------------------
    % Safety check
    % ------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        return;
    end

    msg = [];

    % ------------------------------------------------------------
    % Try cached ROS message
    % ------------------------------------------------------------
    if isfield(zed.pCom, "lastMsgInfo") && ~isempty(zed.pCom.lastMsgInfo)
        msg = zed.pCom.lastMsgInfo;
    end

    % ------------------------------------------------------------
    % Blocking receive (only if needed)
    % ------------------------------------------------------------
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
    % Parse CameraInfo
    % ------------------------------------------------------------
    try
        calib = parseCameraInfoMessage(msg);

        % Cache results
        zed.pData.Calibration = calib;
        zed.pFlag.HasCalibration = true;

    catch excp
        zed.pFlag.LastError = "CameraInfo parse failed: " + excp.message;
        % mantém calib vazio (shape conhecido)
    end
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

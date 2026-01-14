function calib = rGetCalibration(zed)
%rGetCalibration Get camera intrinsics/extrinsics from ROS2 CameraInfo.
%
% Returns a struct with:
%   K (3x3), D (Nx1), R (3x3), P (3x4),
%   Width, Height, DistortionModel

    calib = struct();

    % ------------------------------------------------------------
    % FAST PATH: return cached calibration if already available
    % ------------------------------------------------------------
    if zed.pFlag.HasCalibration && ~isempty(fieldnames(zed.pData.Calibration))
        calib = zed.pData.Calibration;
        return;
    end

    % ------------------------------------------------------------
    % Safety check
    % ------------------------------------------------------------
    if ~zed.pFlag.Connected
        zed.pFlag.LastError = 'Not connected. Call rConnect() first.';
        return;
    end

    msg = [];

    % ------------------------------------------------------------
    % Try cached ROS message
    % ------------------------------------------------------------
    if isfield(zed.pCom, 'lastMsgInfo') && ~isempty(zed.pCom.lastMsgInfo)
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
            zed.pFlag.LastError = ['CameraInfo receive failed: ' excp.message];
            return;
        end
    end

    % ------------------------------------------------------------
    % Parse CameraInfo
    % ------------------------------------------------------------
    try
        % CameraInfo fields:
        % msg.k (9), msg.d (N), msg.r (9), msg.p (12)
        K = reshape(double(msg.k), [3, 3])';
        R = reshape(double(msg.r), [3, 3])';
        P = reshape(double(msg.p), [3, 4])';

        calib.K = K;
        calib.D = double(msg.d(:));
        calib.R = R;
        calib.P = P;

        calib.Width = double(msg.width);
        calib.Height = double(msg.height);
        calib.DistortionModel = char(msg.distortion_model);

        zed.pData.Calibration = calib;
        zed.pFlag.HasCalibration = true;

    catch excp
        zed.pFlag.LastError = ['CameraInfo parse failed: ' excp.message];
    end
end

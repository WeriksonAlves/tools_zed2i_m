function [calib, intr, ok, err] = rosGetCalibration(zed, varargin)
%rosGetCalibration Decode CameraInfo into calibration struct (decode-only).
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    calib = buildEmptyCalibrationStruct();
    intr = [];
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subInfo") || isempty(zed.pCom.subInfo)
        err = "CameraInfo subscriber not initialized (subInfo).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subInfo, zed.pPar.timeoutSec);
            zed.pCom.lastMsgInfo = msg;
        catch excp
            err = "CameraInfo receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgInfo")
            msg = zed.pCom.lastMsgInfo;
        end
        if isempty(msg)
            err = "CameraInfo cache empty (no lastMsgInfo).";
            return;
        end
    end

    try
        calib = parseCameraInfoMessage(msg);
        intr = safeParseIntrinsics(calib);
        ok = true;
    catch excp
        err = "CameraInfo decode failed: " + string(excp.message);
    end
end

function calib = buildEmptyCalibrationStruct()
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

function calib = parseCameraInfoMessage(msg)
    calib = buildEmptyCalibrationStruct();

    calib.K = reshape(double(msg.k), [3, 3])';
    calib.R = reshape(double(msg.r), [3, 3])';
    calib.P = reshape(double(msg.p), [3, 4])';

    calib.D = double(msg.d(:));
    calib.Width = double(msg.width);
    calib.Height = double(msg.height);
    calib.DistortionModel = char(msg.distortion_model);
end

function intr = safeParseIntrinsics(calib)
    intr = [];

    if ~isstruct(calib) || isempty(calib.K) || isempty(calib.Width) || isempty(calib.Height)
        return;
    end

    try
        fx = calib.K(1, 1);
        fy = calib.K(2, 2);
        cx = calib.K(1, 3);
        cy = calib.K(2, 3);

        intr = cameraIntrinsics([fx fy], [cx cy], [calib.Height calib.Width]);
    catch
        intr = [];
    end
end

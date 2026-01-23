function pc = rGetPointCloud(zed)
%rGetPointCloud Get registered point cloud from ROS2 sensor_msgs/PointCloud2.
%
% Returns a struct with stable fields:
%   Timestamp  (datetime)
%   FrameId    (string)
%   XYZ        (Nx3 double) [x y z] in meters
%   HasColor   (logical)
%   Color      (Nx3 uint8) or [] if not available

    % Não alocamos struct completo aqui; só em caso de erro ou no parse.
    pc = struct();

    % ---------------------------------------------------------------------
    % Sanity checks
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call lcConnect() first.";
        zed.pFlag.HasPointCloud = false;
        pc = buildEmptyPointCloudStruct();
        return;
    end

    if ~isfield(zed.pCom, "subPointCloud") || isempty(zed.pCom.subPointCloud)
        zed.pFlag.LastError = ...
            "PointCloud subscriber not initialized. Enable point cloud (enablePointCloud=true) before lcConnect().";
        zed.pFlag.HasPointCloud = false;
        pc = buildEmptyPointCloudStruct();
        return;
    end

    % ---------------------------------------------------------------------
    % Try to receive a fresh message (with cached fallback)
    % ---------------------------------------------------------------------
    [msg, ok, lastErr] = receiveWithCache( ...
        zed.pCom.subPointCloud, ...
        getFieldOrEmpty(zed.pCom, "lastMsgPointCloud"), ...
        zed.pPar.timeoutSec, ...
        "PointCloud");

    if ~ok
        zed.pFlag.LastError = lastErr;
        zed.pFlag.HasPointCloud = false;
        pc = buildEmptyPointCloudStruct();
        return;
    end

    zed.pCom.lastMsgPointCloud = msg;

    % ---------------------------------------------------------------------
    % Parse PointCloud2 message
    % ---------------------------------------------------------------------
    try
        pc = parsePointCloudMessage(msg);
        zed.pData.PointCloud = pc;
        zed.pFlag.HasPointCloud = true;
        zed.pFlag.LastError = "";
    catch excp
        zed.pFlag.LastError = "PointCloud parse failed: " + excp.message;
        zed.pFlag.HasPointCloud = false;
        pc = buildEmptyPointCloudStruct();
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function pc = buildEmptyPointCloudStruct()
    pc = struct( ...
        "Timestamp", datetime('now'), ...
        "FrameId", "", ...
        "XYZ", zeros(0, 3), ...
        "HasColor", false, ...
        "Color", zeros(0, 3, 'uint8') ...
    );
end

function flag = safeFlag(flags, fieldName)
    flag = false;
    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

function value = getFieldOrEmpty(s, fieldName)
%getFieldOrEmpty Retrieve struct field or [] if not present.
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = [];
    end
end

function [msgOut, ok, errMsg] = receiveWithCache(sub, lastMsg, timeoutSec, label)
%receiveWithCache Try to receive a fresh ROS2 message with cached fallback.
%
%   [msgOut, ok, errMsg] = receiveWithCache(sub, lastMsg, timeoutSec, label)
%
%   ok      : true if either a new message or a cached one is returned
%   errMsg  : non-empty only when no message is available

    msgOut = [];
    ok = false;
    errMsg = "";

    try
        msgOut = receive(sub, timeoutSec);
        ok = true;
        return;
    catch excp
        if ~isempty(lastMsg)
            msgOut = lastMsg;
            ok = true;
            errMsg = "";
        else
            errMsg = label + " receive failed: " + excp.message;
        end
    end
end

function pc = parsePointCloudMessage(msg)
%parsePointCloudMessage Decode sensor_msgs/PointCloud2 into a stable struct.

    % Timestamp / frame
    ts = datetime('now');
    frameId = "";

    if isfield(msg, "header")
        if isfield(msg.header, "frame_id")
            frameId = string(msg.header.frame_id);
        end
        % Se quiser no futuro, pode converter header.stamp para tempo real.
    end

    % XYZ in meters (custo dominante aqui)
    xyz = rosReadXYZ(msg);

    % Cor (desativada por enquanto; mantém API preparada)
    hasColor = false;
    color = zeros(0, 3, 'uint8');

    pc = struct( ...
        "Timestamp", ts, ...
        "FrameId", frameId, ...
        "XYZ", xyz, ...
        "HasColor", hasColor, ...
        "Color", color ...
    );
end

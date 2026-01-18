function pc = rGetPointCloud(zed)
%rGetPointCloud Get registered point cloud from ROS2 sensor_msgs/PointCloud2.
%
% Returns a struct with stable fields:
%   Timestamp  (datetime)
%   FrameId    (string)
%   XYZ        (Nx3 double) [x y z] in meters
%   HasColor   (logical)
%   Color      (Nx3 uint8) or [] if not available
%
% This is an optional feature controlled by pPar.enablePointCloud.

    pc = buildEmptyPointCloudStruct();

    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        return;
    end

    if ~isfield(zed.pCom, "subPointCloud") || isempty(zed.pCom.subPointCloud)
        zed.pFlag.LastError = ...
            "PointCloud subscriber not initialized. Enable point cloud (pPar.enablePointCloud=true or constructor) before rConnect().";
        return;
    end

    msg = [];
    hasNewMsg = false;

    % ---------------------------------------------------------------------
    % Always try to receive a fresh message
    % ---------------------------------------------------------------------
    try
        msg = receive(zed.pCom.subPointCloud, zed.pPar.timeoutSec);
        zed.pCom.lastMsgPointCloud = msg;
        hasNewMsg = true;
    catch excp
        % Fallback: use last valid message if available
        if isfield(zed.pCom, "lastMsgPointCloud") && ~isempty(zed.pCom.lastMsgPointCloud)
            msg = zed.pCom.lastMsgPointCloud;
        else
            zed.pFlag.LastError = "PointCloud receive failed: " + excp.message;
            zed.pFlag.HasPointCloud = false;
            return;
        end
    end

    % ---------------------------------------------------------------------
    % Parse PointCloud2 message
    % ---------------------------------------------------------------------
    try
        pc = parsePointCloudMessage(msg);

        % Cache decoded snapshot
        zed.pData.PointCloud = pc;
        zed.pFlag.HasPointCloud = true;

        if hasNewMsg
            zed.pFlag.LastError = "";
        end

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

function pc = parsePointCloudMessage(msg)
    pc = buildEmptyPointCloudStruct();
    pc.Timestamp = datetime('now');

    % Frame ID (se disponível)
    if isfield(msg, "header") && isfield(msg.header, "frame_id")
        pc.FrameId = string(msg.header.frame_id);
    end

    % XYZ em metros
    xyz = rosReadXYZ(msg);       % Robotics System Toolbox
    pc.XYZ = xyz;

    % Opcional: cor (se você quiser habilitar depois com rosReadRGB)
    pc.HasColor = false;
    pc.Color = zeros(0, 3, 'uint8');
end

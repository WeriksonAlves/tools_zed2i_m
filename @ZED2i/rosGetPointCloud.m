function [pc, ok, err] = rosGetPointCloud(zed, varargin)
%rosGetPointCloud Decode PointCloud2 from cached or freshly received message.
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    pc = struct();
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subPointCloud") || isempty(zed.pCom.subPointCloud)
        err = "PointCloud subscriber not initialized (subPointCloud).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subPointCloud, zed.pPar.timeoutSec);
            zed.pCom.lastMsgPointCloud = msg;
        catch excp
            err = "PointCloud receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgPointCloud")
            msg = zed.pCom.lastMsgPointCloud;
        end
        if isempty(msg)
            err = "PointCloud cache empty (no lastMsgPointCloud).";
            return;
        end
    end

    try
        pc = parsePointCloudMessage(msg);
        ok = true;
    catch excp
        err = "PointCloud decode failed: " + string(excp.message);
    end
end

function pc = parsePointCloudMessage(msg)
    frameId = "";
    if isfield(msg, "header") && isfield(msg.header, "frame_id")
        frameId = string(msg.header.frame_id);
    end

    xyz = rosReadXYZ(msg); % heavy op; only called when getSensorData decides

    pc = struct( ...
        "Timestamp", datetime('now'), ...
        "FrameId", frameId, ...
        "XYZ", xyz, ...
        "HasColor", false, ...
        "Color", zeros(0, 3, 'uint8') ...
    );
end

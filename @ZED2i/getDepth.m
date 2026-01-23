function [depth, validMask] = getDepth(zed)
%getDepth Blocking receive + decode of the depth stream with sanitization.
%
%   [depth, validMask] = zed.getDepth()
%
% depth     : depth frame (single, meters), or [] if unavailable
% validMask : logical mask of valid pixels, or [] if depth empty

    depth = [];
    validMask = [];

    % ---------------------------------------------------------------------
    % Sanity checks
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call lcConnect() first.";
        zed.pFlag.HasDepth = false;
        return;
    end

    if ~isfield(zed.pCom, "subDepth") || isempty(zed.pCom.subDepth)
        zed.pFlag.LastError = ...
            "Depth subscriber not initialized. Check topics and lcConnect().";
        zed.pFlag.HasDepth = false;
        return;
    end

    % ---------------------------------------------------------------------
    % Blocking receive + decode + sanitization
    % ---------------------------------------------------------------------
    try
        msgDepth = receive(zed.pCom.subDepth, zed.pPar.timeoutSec);
        zed.pCom.lastMsgDepth = msgDepth;

        depthRaw = rosReadImage(msgDepth);

        depth = zed.utilSanitizeDepth(depthRaw);
        validMask = isfinite(depth) & (depth > 0);

        zed.pData.Depth     = depth;
        zed.pFlag.HasDepth  = true;
        zed.pFlag.LastError = "";

        zed.utilUpdateFps("depth");

    catch excp
        if isfield(zed.pData, "Metrics") && isfield(zed.pData.Metrics, "DepthDrops")
            zed.pData.Metrics.DepthDrops = zed.pData.Metrics.DepthDrops + 1;
        end

        zed.pFlag.HasDepth  = false;
        zed.pFlag.LastError = "Depth receive failed: " + string(excp.message);
        depth = [];
        validMask = [];
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function flag = safeFlag(flags, fieldName)
%safeFlag Return logical flag if available, otherwise false.

    flag = false;
    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

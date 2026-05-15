function [depth, validMask, ok, err] = rosGetDepth(zed, varargin)
%rosGetDepth Decode depth from cached or freshly received message.
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    depth = [];
    validMask = [];
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subDepth") || isempty(zed.pCom.subDepth)
        err = "Depth subscriber not initialized (subDepth).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subDepth, zed.pPar.timeoutSec);
            zed.pCom.lastMsgDepth = msg;
        catch excp
            err = "Depth receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgDepth")
            msg = zed.pCom.lastMsgDepth;
        end
        if isempty(msg)
            err = "Depth cache empty (no lastMsgDepth).";
            return;
        end
    end

    try
        depthRaw = rosReadImage(msg);
        depth = zed.sSanitizeDepth(depthRaw);
        validMask = isfinite(depth) & (depth > 0);
        ok = true;
    catch excp
        err = "Depth decode failed: " + string(excp.message);
    end
end

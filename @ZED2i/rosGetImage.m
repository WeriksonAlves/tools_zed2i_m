function [img, ok, err] = rosGetImage(zed, varargin)
%rosGetImage Decode RGB image from cached or freshly received message.
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    img = [];
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subImage") || isempty(zed.pCom.subImage)
        err = "Image subscriber not initialized (subImage).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subImage, zed.pPar.timeoutSec);
            zed.pCom.lastMsgImage = msg;
        catch excp
            err = "Image receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgImage")
            msg = zed.pCom.lastMsgImage;
        end
        if isempty(msg)
            err = "Image cache empty (no lastMsgImage).";
            return;
        end
    end

    try
        img = rosReadImage(msg);
        ok = true;
    catch excp
        err = "Image decode failed: " + string(excp.message);
    end
end

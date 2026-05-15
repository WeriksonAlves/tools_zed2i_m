function [msg, ok, err] = sGetCachedMsg(zed, subField, lastMsgField, label, mode)
%sGetCachedMsg Get latest message from cache, optionally forcing receive().
%
% mode:
%   "cached" -> return cache only (fast, non-blocking)
%   "fetch"  -> try receive() and update cache; fallback to cache

    msg = [];
    ok = false;
    err = "";

    mode = string(mode);

    % 1) Cached fast path
    if isfield(zed.pCom, lastMsgField) && ~isempty(zed.pCom.(lastMsgField))
        msg = zed.pCom.(lastMsgField);
        ok = true;
        if mode == "cached"
            return;
        end
    end

    % 2) If fetch requested, try to receive fresh message
    if mode ~= "fetch"
        if ~ok
            err = label + " not available (cache empty).";
        end
        return;
    end

    if ~isfield(zed.pCom, subField) || isempty(zed.pCom.(subField))
        err = label + " subscriber not initialized. Check rosConnect().";
        return;
    end

    try
        msg = receive(zed.pCom.(subField), zed.pPar.timeoutSec);
        zed.pCom.(lastMsgField) = msg;
        ok = true;
        err = "";
    catch excp
        % fallback to cache if exists
        if isfield(zed.pCom, lastMsgField) && ~isempty(zed.pCom.(lastMsgField))
            msg = zed.pCom.(lastMsgField);
            ok = true;
            err = "";
        else
            err = label + " receive failed: " + string(excp.message);
        end
    end
end

function hz = sGetStreamHz(zed, streamName)
%sGetStreamHz Return configured stream rate (Hz) for a given stream.
%
% If missing, returns 0 (disabled).

    hz = 0;

    if ~isfield(zed, "pPar") || ~isstruct(zed.pPar)
        return;
    end
    if ~isfield(zed.pPar, "streamHz") || ~isstruct(zed.pPar.streamHz)
        return;
    end
    if ~isfield(zed.pPar.streamHz, streamName)
        return;
    end

    hz = double(zed.pPar.streamHz.(streamName));
    if isnan(hz) || hz < 0
        hz = 0;
    end
end

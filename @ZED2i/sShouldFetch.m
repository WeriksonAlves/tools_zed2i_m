function tf = sShouldFetch(zed, streamName)
%sShouldFetch Decide if a stream should be fetched now based on streamHz.
%
% Rules:
%   - Hz <= 0  -> disabled (never fetch)
%   - Hz = inf -> always fetch
%   - Otherwise, fetch if elapsed >= 1/Hz
%
% Scheduler state is stored in:
%   zed.pData.Scheduler.lastTic.(streamName) = tic handle

    hz = zed.sGetStreamHz(streamName);

    if hz <= 0
        tf = false;
        return;
    end
    if isinf(hz)
        tf = true;
        return;
    end

    % init storage
    if ~isfield(zed.pData, "Scheduler") || ~isstruct(zed.pData.Scheduler)
        zed.pData.Scheduler = struct();
    end
    if ~isfield(zed.pData.Scheduler, "lastTic") || ~isstruct(zed.pData.Scheduler.lastTic)
        zed.pData.Scheduler.lastTic = struct();
    end

    field = string(streamName);

    if ~isfield(zed.pData.Scheduler.lastTic, field) || isempty(zed.pData.Scheduler.lastTic.(field))
        tf = true; % first time
        return;
    end

    elapsed = toc(zed.pData.Scheduler.lastTic.(field));
    tf = elapsed >= (1 / hz);
end

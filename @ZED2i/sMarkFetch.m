function sMarkFetch(zed, streamName)
%sMarkFetch Mark a stream as fetched now (tic reset).
%
% Scheduler state is stored in zed.pData.Scheduler.lastTic.(streamName).

    if ~isfield(zed.pData, "Scheduler") || ~isstruct(zed.pData.Scheduler)
        zed.pData.Scheduler = struct();
    end
    if ~isfield(zed.pData.Scheduler, "lastTic") || ~isstruct(zed.pData.Scheduler.lastTic)
        zed.pData.Scheduler.lastTic = struct();
    end

    field = char(string(streamName));
    zed.pData.Scheduler.lastTic.(field) = tic;
end

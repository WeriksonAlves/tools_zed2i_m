function t = sNowSec(zed)
%sNowSec Fast timestamp in seconds (double) without datetime overhead.
%
% Uses a persistent tic base per object instance.

    arguments
        zed (1, 1)
    end

    if ~isfield(zed.pData, "BaseTic") || isempty(zed.pData.BaseTic)
        zed.pData.BaseTic = tic;
        t = 0.0;
        return;
    end

    t = toc(zed.pData.BaseTic);
end

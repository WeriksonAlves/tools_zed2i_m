function metrics = sSafeGetMetrics(zed)
%sSafeGetMetrics Stable metrics output.

    metrics = struct("ImageFps", 0, "DepthFps", 0, "ImageDrops", 0, "DepthDrops", 0);

    if isstruct(zed.pData) && isfield(zed.pData, "Metrics") && isstruct(zed.pData.Metrics)
        src = zed.pData.Metrics;
        metrics = copyIfExists(metrics, src, "ImageFps");
        metrics = copyIfExists(metrics, src, "DepthFps");
        metrics = copyIfExists(metrics, src, "ImageDrops");
        metrics = copyIfExists(metrics, src, "DepthDrops");
    end
end

function dst = copyIfExists(dst, src, fieldName)
    if isstruct(src) && isfield(src, fieldName)
        dst.(fieldName) = src.(fieldName);
    end
end

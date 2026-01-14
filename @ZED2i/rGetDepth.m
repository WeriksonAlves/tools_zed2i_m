function [depth, validMask] = rGetDepth(zed)
%rGetDepth Return the last decoded depth buffer and a validity mask.

    depth = zed.pData.Depth;
    validMask = isfinite(depth) & (depth > 0);
end

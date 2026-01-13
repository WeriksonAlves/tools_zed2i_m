function depth = rGetDepth(zed)
%rGetDepth Return the last decoded depth buffer.

    depth = zed.pData.Depth;
end
function depthOut = mAuxSanitizeDepth(~, depthIn)
%mAuxSanitizeDepth Normalize depth values: invalid to NaN and single precision.
%
%   depthOut = zed.mAuxSanitizeDepth(depthRaw);

    depthOut = depthIn;

    % Uma única varredura: elimina não finitos e valores <= 0
    invalidMask = ~isfinite(depthOut) | (depthOut <= 0);
    depthOut(invalidMask) = NaN;

    % Mantém como single para reduzir footprint
    if ~isa(depthOut, "single")
        depthOut = single(depthOut);
    end
end

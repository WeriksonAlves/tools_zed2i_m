function value = sSafeFlag(zed, fieldName)
%sSafeFlag Read a boolean flag from zed.pFlag safely.

    value = false;
    if isstruct(zed.pFlag) && isfield(zed.pFlag, fieldName)
        value = logical(zed.pFlag.(fieldName));
    end
end

function err = sSafeLastError(zed)
%sSafeLastError Return the last error string if available.

    err = "";
    if isstruct(zed.pFlag) && isfield(zed.pFlag, "LastError")
        err = string(zed.pFlag.LastError);
    end
end

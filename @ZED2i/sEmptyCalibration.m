function calib = sEmptyCalibration(~)
%sEmptyCalibration Persistent empty calibration struct template.
%
% Contract (Model 1):
%   - Calibration is a single struct that ALSO includes intrinsics (if available).
%   - The cameraIntrinsics object (or []) is stored in field "Intrinsics".

    persistent EMPTY_CALIB
    if isempty(EMPTY_CALIB)
        EMPTY_CALIB = struct( ...
            "K", [], "D", [], "R", [], "P", [], ...
            "Width", [], "Height", [], ...
            "DistortionModel", "", ...
            "Intrinsics", [] ...
        );
    end
    calib = EMPTY_CALIB;
end

function intr = rGetIntrinsics(zed)
%rGetIntrinsics Return MATLAB cameraIntrinsics from CameraInfo K and size.
%
% Requires Computer Vision Toolbox.

    calib = zed.rGetCalibration();
    if isempty(fieldnames(calib))
        intr = [];
        return;
    end

    fx = calib.K(1,1);
    fy = calib.K(2,2);
    cx = calib.K(1,3);
    cy = calib.K(2,3);

    % MATLAB expects [fx fy] and principal point [cx cy]
    focalLength = [fx fy];
    principalPoint = [cx cy];
    imageSize = [calib.Height calib.Width]; % [rows cols]

    intr = cameraIntrinsics(focalLength, principalPoint, imageSize);
end

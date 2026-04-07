function iParameters(zed, camId)
%iParameters Initialize parameter struct (pPar) with safe defaults.

    if nargin < 2 || isempty(camId)
        if isstruct(zed.pPar) && isfield(zed.pPar, "camId") && ~isempty(zed.pPar.camId)
            camId = zed.pPar.camId;
        else
            camId = 0;
        end
    end

    zed.pPar = struct();
    zed.pPar.camId = double(camId);

    zed.pPar.timeoutSec = 1.0;
    zed.pPar.UseCallbacks = true;


    zed.pPar.streamHz = struct( ...
        "image", 15, ...
        "depth", 15, ...
        "imu",   100, ...
        "pose",  15, ...
        "pcd",   1, ...
        "calib", 0 ...
    );

    zed.pPar.enableImu        = true;
    zed.pPar.enablePose       = true;
    zed.pPar.enablePointCloud = false;

    ns = "/zed/zed_node";
    zed.pPar.topicImage      = ns + "/rgb/image_rect_color";
    zed.pPar.topicDepth      = ns + "/depth/depth_registered";
    zed.pPar.topicImu        = ns + "/imu/data";
    zed.pPar.topicOdom       = ns + "/odom";
    zed.pPar.topicPointCloud = ns + "/point_cloud/cloud_registered";
    zed.pPar.topicCameraInfo = ns + "/rgb/camera_info";

    zed.pPar.fpsAlpha = 0.1;
end

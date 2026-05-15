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

    % ---------------------------------------------------------------------
    % CAD model defaults
    % ---------------------------------------------------------------------
    theta = -pi / 2;

    Rz = [ ...
        cos(theta), -sin(theta), 0; ...
        sin(theta),  cos(theta), 0; ...
        0,           0,          1 ...
    ];

    zed.pPar.cad = struct();
    zed.pPar.cad.enable = true;
    zed.pPar.cad.scale = 0.0100028;
    zed.pPar.cad.center = true;
    zed.pPar.cad.useSingle = true;
    zed.pPar.cad.R_model_to_body = Rz;
    zed.pPar.cad.parts = struct( ...
        "obj", "ZED2i.obj", ...
        "mtl", "ZED2i.mtl", ...
        "t_body", [0; 0; 0] ...
    );

    zed.pPar.fpsAlpha = 0.1;
end

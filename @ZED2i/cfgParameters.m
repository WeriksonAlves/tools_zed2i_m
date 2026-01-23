function cfgParameters(zed)
%cfgParameters Define default parameters for the ZED2i MATLAB ROS2 wrapper.
%
% This method is responsible only for setting configuration values
% (topics, timeouts, behavior flags). No ROS calls are made here.

    % ---------------------------------------------------------------------
    % ROS behavior
    % ---------------------------------------------------------------------
    zed.pPar.fps = 1/30;  % default expected FPS

    zed.pPar.fpsAlpha = 0.2;

    zed.pPar.timeoutSec = 2.0;

    % ---------------------------------------------------------------------
    % Node name
    % ---------------------------------------------------------------------
    zed.pPar.nodeName = "zed2i_matlab_node";

    % Legacy/ROS1-related fields (kept for lab compatibility; not used in ROS2)
    zed.pPar.rosMasterURI  = "http://localhost:11311";
    zed.pPar.nodeNamespace = "/zed";

    % ---------------------------------------------------------------------
    % Core ROS 2 topics (currently used by the implementation)
    % ---------------------------------------------------------------------
    zed.pPar.topicImage      = "/zed/zed_node/rgb/image_rect_color";
    zed.pPar.topicDepth      = "/zed/zed_node/depth/depth_registered";
    zed.pPar.topicCameraInfo = "/zed/zed_node/rgb/camera_info";

    % Right image kept for potential future extensions (stereo use cases)
    zed.pPar.topicImageLeft  = "/zed/zed_node/left/image_rect_color";
    zed.pPar.topicImageRight = "/zed/zed_node/right/image_rect_color";

    % ---------------------------------------------------------------------
    % Optional topics (IMU / Pose / PointCloud)
    % ---------------------------------------------------------------------
    zed.pPar.topicImu  = "/zed/zed_node/imu/data";
    zed.pPar.enableImu = false;

    zed.pPar.topicOdom  = "/zed/zed_node/odom";
    zed.pPar.topicPose  = "/zed/zed_node/pose";
    zed.pPar.enablePose = false;

    zed.pPar.topicPointCloud      = "/zed/zed_node/point_cloud/cloud_registered";
    zed.pPar.enablePointCloud     = false;   % default: off
    zed.pPar.autoFetchPointCloud  = false;   % NEW: rGetSensorData não faz fetch pesado por padrão

    

    
end

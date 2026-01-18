function iParameters(zed)
%iParameters Define default parameters for the ZED2i MATLAB ROS2 wrapper.
%
% This method is responsible only for setting configuration values
% (topics, timeouts, behavior flags). No ROS calls are made here.

    % ---------------------------------------------------------------------
    % Core ROS 2 topics (currently used by the implementation)
    % ---------------------------------------------------------------------
    zed.pPar.topicImage      = "/zed/zed_node/left/image_rect_color";
    zed.pPar.topicDepth      = "/zed/zed_node/depth/depth_registered";
    zed.pPar.topicCameraInfo = "/zed/zed_node/left/camera_info";

    % Right image kept for potential future extensions (stereo use cases)
    zed.pPar.topicImageRight = "/zed/zed_node/right/image_rect_color";

    % ---------------------------------------------------------------------
    % Optional topics (not used yet in the current implementation)
    % Kept for alignment with laboratory conventions and future work.
    % ---------------------------------------------------------------------
    zed.pPar.topicPointCloud = "/zed/zed_node/point_cloud/cloud_registered";
    zed.pPar.enablePointCloud = true;

    % Optional topics (not used yet in the current implementation)
    zed.pPar.topicImu  = "/zed/zed_node/imu/data";
    zed.pPar.enableImu = false;  % keep default false for v1.0.0 compatibility


    zed.pPar.topicOdom  = "/zed/zed_node/odom";
    zed.pPar.topicPose  = "/zed/zed_node/pose";
    zed.pPar.enablePose = false;

    % Legacy/ROS1-related fields (kept for lab compatibility; not used in ROS2)
    zed.pPar.rosMasterURI  = "http://localhost:11311";
    zed.pPar.nodeNamespace = "/zed";

    % ---------------------------------------------------------------------
    % ROS behavior
    % ---------------------------------------------------------------------
    % Timeout used in receive() calls (rGrab, rGetCalibration, etc.)
    zed.pPar.timeoutSec = 2.0;

    % Exponential moving average smoothing factor for FPS estimation.
    % Used in rGrab/updateFps.
    zed.pPar.fpsAlpha = 0.2;

    % ---------------------------------------------------------------------
    % Node name
    % ---------------------------------------------------------------------
    zed.pPar.nodeName = "zed2i_matlab_node";
end

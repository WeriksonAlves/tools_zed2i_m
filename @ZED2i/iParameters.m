function iParameters(zed)
%iParameters Initialize default parameters for ZED2i ROS2 topics.

    zed.pPar.namespace = '/zed/zed_node';

    % Core topics (from your ros2 topic list)
    zed.pPar.topicImage      = '/zed/zed_node/left/image_rect_color';
    zed.pPar.topicCameraInfo = '/zed/zed_node/left/camera_info';
    zed.pPar.topicDepth      = '/zed/zed_node/depth/depth_registered';

    % Optional
    zed.pPar.topicPointCloud = '/zed/zed_node/point_cloud/cloud_registered';
    zed.pPar.enablePointCloud = true;

    % ROS behavior
    zed.pPar.timeoutSec = 2.0;   % receive() timeout used in rGrab (when needed)

    % Node name
    zed.pPar.nodeName = 'zed2i_matlab_node';
end

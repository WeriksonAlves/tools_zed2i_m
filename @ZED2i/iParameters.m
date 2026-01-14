function iParameters(zed)
%iParameters Default parameters for the minimal ZED2i wrapper.

    % Topics
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

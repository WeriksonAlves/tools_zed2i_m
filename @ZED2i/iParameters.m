function iParameters(zed)
%iParameters Define default parameters for the ZED2i MATLAB ROS2 wrapper.
%
% This method is responsible only for setting configuration values
% (topics, timeouts, behavior flags). No ROS calls are made here.

    % ---------------------------------------------------------------------
    % Parâmetros gerais
    % ---------------------------------------------------------------------
    % FPS esperado (não é usado diretamente na lógica atual, mas pode ser
    % útil para validações ou futuros ajustes de timing).
    zed.pPar.fps = 1/30;

    % Exponential moving average smoothing factor for FPS estimation.
    % Used in updateFps.
    zed.pPar.fpsAlpha = 0.2;

    % Timeout usado em receive() (image, depth, calib, IMU, pose, etc.)
    zed.pPar.timeoutSec = 2.0;

    % Nome do nó ROS2
    zed.pPar.nodeName = "zed2i_matlab_node";

    % Legacy/ROS1-related fields (mantidos por compatibilidade de lab)
    zed.pPar.rosMasterURI  = "http://localhost:11311";
    zed.pPar.nodeNamespace = "/zed";

    % ---------------------------------------------------------------------
    % Tópicos de imagem e profundidade
    % ---------------------------------------------------------------------
    zed.pPar.topicImage      = "/zed/zed_node/rgb/image_rect_color";
    zed.pPar.topicDepth      = "/zed/zed_node/depth/depth_registered";
    zed.pPar.topicCameraInfo = "/zed/zed_node/rgb/camera_info";

    % Right image kept for potential future extensions (stereo use cases)
    zed.pPar.topicImageLeft  = "/zed/zed_node/left/image_rect_color";
    zed.pPar.topicImageRight = "/zed/zed_node/right/image_rect_color";

    % ---------------------------------------------------------------------
    % IMU
    % ---------------------------------------------------------------------
    zed.pPar.topicImu  = "/zed/zed_node/imu/data";
    % Mantido como false para compatibilidade v1.0.0; ativado via construtor
    % ou ajuste explícito pelo usuário:
    %   zed = ZED2i("enableImu", true);
    zed.pPar.enableImu = false;

    % ---------------------------------------------------------------------
    % Pose/Odom
    % ---------------------------------------------------------------------
    zed.pPar.topicOdom  = "/zed/zed_node/odom";
    % topicPose mantido para futuros usos (ex.: geometry_msgs/PoseStamped)
    zed.pPar.topicPose  = "/zed/zed_node/pose";
    zed.pPar.enablePose = false;  % default: off

    % ---------------------------------------------------------------------
    % PointCloud
    % ---------------------------------------------------------------------
    zed.pPar.topicPointCloud  = "/zed/zed_node/point_cloud/cloud_registered";
    % default: off para não pesar em setups básicos; ativar explicitamente
    zed.pPar.enablePointCloud = false;
end

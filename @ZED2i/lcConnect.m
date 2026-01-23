function lcConnect(zed)
%lcConnect Create ROS2 node and image/depth/camera_info subscribers.
%
% This method is idempotent: if the object is already connected,
% it returns immediately without side effects.

    % Reset last error at the beginning of the operation
    zed.pFlag.LastError = "";

    % Fast path: already connected, nothing to do
    if isfield(zed.pFlag, "Connected") && zed.pFlag.Connected
        return;
    end

    % ---------------------------------------------------------------------
    % Validate minimal configuration before touching ROS
    % ---------------------------------------------------------------------
    if ~isfield(zed.pPar, "nodeName") || strlength(string(zed.pPar.nodeName)) == 0
        zed.pFlag.LastError = "Invalid nodeName in pPar.";
        error("ZED2i:lcConnect:InvalidNodeName", zed.pFlag.LastError);
    end

    requiredTopics = { ...
        "topicImage",      "sensor_msgs/Image"; ...
        "topicDepth",      "sensor_msgs/Image"; ...
        "topicCameraInfo", "sensor_msgs/CameraInfo" ...
    };

    for k = 1:size(requiredTopics, 1)
        fieldName = requiredTopics{k, 1};
        if ~isfield(zed.pPar, fieldName) || strlength(string(zed.pPar.(fieldName))) == 0
            zed.pFlag.LastError = "Missing or empty topic configuration: " + fieldName;
            error("ZED2i:lcConnect:InvalidTopicConfig", zed.pFlag.LastError);
        end
    end

    % ---------------------------------------------------------------------
    % Clean any previous partial state to avoid leaks or inconsistent flags
    % ---------------------------------------------------------------------
    zed.utilResetCommState();

    try
        % Create ROS2 node
        zed.pCom.node = ros2node(zed.pPar.nodeName);

        % Required subscribers
        zed.pCom.subImage = createSubscriber( ...
            zed, zed.pPar.topicImage, "sensor_msgs/Image");

        zed.pCom.subDepth = createSubscriber( ...
            zed, zed.pPar.topicDepth, "sensor_msgs/Image");

        zed.pCom.subInfo = createSubscriber( ...
            zed, zed.pPar.topicCameraInfo, "sensor_msgs/CameraInfo");

        % Optional IMU subscriber
        if isfield(zed.pPar, "enableImu") && zed.pPar.enableImu
            zed.pCom.subImu = createSubscriber( ...
                zed, zed.pPar.topicImu, "sensor_msgs/Imu");
            zed.pFlag.HasImu = false;
        end

        % Optional Pose subscriber
        if isfield(zed.pPar, "enablePose") && zed.pPar.enablePose
            zed.pCom.subOdom = createSubscriber( ...
                zed, zed.pPar.topicOdom, "nav_msgs/Odometry");
            zed.pFlag.HasPose = false;
        end

        % Optional PointCloud subscriber
        if isfield(zed.pPar, "enablePointCloud") && zed.pPar.enablePointCloud
            zed.pCom.subPointCloud = createSubscriber( ...
                zed, zed.pPar.topicPointCloud, "sensor_msgs/PointCloud2");
            zed.pFlag.HasPointCloud = false;
        end

        % Initialize flags after successful connection
        zed.pFlag.Connected      = true;
        zed.pFlag.HasImage       = false;
        zed.pFlag.HasDepth       = false;
        zed.pFlag.HasCalibration = false;

    catch excp
        % Ensure a consistent disconnected state on failure
        zed.pFlag.Connected      = false;
        zed.pFlag.HasImage       = false;
        zed.pFlag.HasDepth       = false;
        zed.pFlag.HasCalibration = false;

        if isfield(zed.pFlag, "HasImu")
            zed.pFlag.HasImu = false;
        end
        if isfield(zed.pFlag, "HasPose")
            zed.pFlag.HasPose = false;
        end
        if isfield(zed.pFlag, "HasPointCloud")
            zed.pFlag.HasPointCloud = false;
        end

        zed.pFlag.LastError = excp.message;

        % Best effort to clean up partially created node/subscribers
        zed.utilResetCommState();

        rethrow(excp);
    end
end

% -------------------------------------------------------------------------
% Local helper
% -------------------------------------------------------------------------

function sub = createSubscriber(zed, topic, msgType)
%createSubscriber Create a ROS2 subscriber with basic validation.

    try
        sub = ros2subscriber(zed.pCom.node, topic, msgType);
    catch excp
        error("ZED2i:lcConnect:SubscriberCreationFailed", ...
            "Failed to create subscriber for topic '%s' (%s): %s", ...
            topic, msgType, excp.message);
    end
end

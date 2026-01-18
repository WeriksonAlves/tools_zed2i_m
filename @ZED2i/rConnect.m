function rConnect(zed)
%rConnect Create ROS2 node and image/depth/camera_info subscribers.
%
% This method is idempotent: if the object is already connected,
% it returns immediately without side effects.

    % Reset last error at the beginning of the operation
    zed.pFlag.LastError = '';

    % Fast path: already connected, nothing to do
    if isfield(zed.pFlag, "Connected") && zed.pFlag.Connected
        return;
    end

    % Validate minimal configuration before touching ROS
    if ~isfield(zed.pPar, "nodeName") || strlength(string(zed.pPar.nodeName)) == 0
        zed.pFlag.LastError = "Invalid nodeName in pPar.";
        error("ZED2i:rConnect:InvalidNodeName", zed.pFlag.LastError);
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
            error("ZED2i:rConnect:InvalidTopicConfig", zed.pFlag.LastError);
        end
    end

    % Clean any previous partial state to avoid leaks or inconsistent flags
    resetCommState(zed);

    try
        % Create ROS2 node
        zed.pCom.node = ros2node(zed.pPar.nodeName);

        % Create subscribers
        zed.pCom.subImage = createSubscriber( ...
            zed, zed.pPar.topicImage, "sensor_msgs/Image");

        zed.pCom.subDepth = createSubscriber( ...
            zed, zed.pPar.topicDepth, "sensor_msgs/Image");

        zed.pCom.subInfo = createSubscriber( ...
            zed, zed.pPar.topicCameraInfo, "sensor_msgs/CameraInfo");

        % Optional subscribers
        if isfield(zed.pPar, "enableImu") && zed.pPar.enableImu
            zed.pCom.subImu = createSubscriber( ...
                zed, zed.pPar.topicImu, "sensor_msgs/Imu");
            zed.pFlag.HasImu = false;
        end

        % Optional subscribers
        if isfield(zed.pPar, "enablePose") && zed.pPar.enablePose
            zed.pCom.subOdom = createSubscriber( ...
                zed, zed.pPar.topicOdom, "nav_msgs/Odometry");
            zed.pFlag.HasPose = false;
        end


        % Initialize flags after successful connection
        zed.pFlag.Connected      = true;
        zed.pFlag.HasImage       = false;
        zed.pFlag.HasDepth       = false;
        zed.pFlag.HasCalibration = false;

    catch excp
        % Ensure a consistent disconnected state on failure
        zed.pFlag.Connected = false;
        zed.pFlag.HasImage = false;
        zed.pFlag.HasDepth = false;
        zed.pFlag.HasCalibration = false;

        zed.pFlag.LastError = excp.message;

        % Best effort to clean up partially created node/subscribers
        resetCommState(zed);

        rethrow(excp);
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function resetCommState(zed)
%resetCommState Clear communication-related fields and subscribers.

    if isfield(zed, "pCom") && ~isempty(zed.pCom)
        % Clear subscribers if they exist
        if isfield(zed.pCom, "subImage") && ~isempty(zed.pCom.subImage)
            clear zed.pCom.subImage;
        end
        if isfield(zed.pCom, "subDepth") && ~isempty(zed.pCom.subDepth)
            clear zed.pCom.subDepth;
        end
        if isfield(zed.pCom, "subInfo") && ~isempty(zed.pCom.subInfo)
            clear zed.pCom.subInfo;
        end

        if isfield(zed.pCom, "subImu") && ~isempty(zed.pCom.subImu)
            clear zed.pCom.subImu;
        end

        if isfield(zed.pCom, "subOdom") && ~isempty(zed.pCom.subOdom)
            clear zed.pCom.subOdom;
        end


        % Clear node if it exists
        if isfield(zed.pCom, "node") && ~isempty(zed.pCom.node)
            clear zed.pCom.node;
        end
    end

    % Reset communication struct to a known baseline
    zed.pCom.node        = [];
    zed.pCom.subImage    = [];
    zed.pCom.subDepth    = [];
    zed.pCom.subInfo     = [];
    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];
    zed.pCom.lastMsgInfo  = [];
    zed.pCom.subImu     = [];
    zed.pCom.lastMsgImu = [];
    zed.pCom.subOdom = [];
    zed.pCom.lastMsgOdom = [];


end

function sub = createSubscriber(zed, topic, msgType)
%createSubscriber Create a ROS2 subscriber with basic validation.

    try
        sub = ros2subscriber(zed.pCom.node, topic, msgType);
    catch excp
        error("ZED2i:rConnect:SubscriberCreationFailed", ...
            "Failed to create subscriber for topic '%s' (%s): %s", ...
            topic, msgType, excp.message);
    end
end

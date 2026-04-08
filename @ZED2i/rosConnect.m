function rosConnect(zed, varargin)
%rosConnect Connect ZED2i object to ROS2 and initialize subscribers.

    % ---------------------------------------------------------------------
    % Defensive init (object-safe). DO NOT use isfield on handle objects.
    % ---------------------------------------------------------------------
    if isempty(zed.pPar) || ~isstruct(zed.pPar) || isempty(fieldnames(zed.pPar))
        % Allow iParameters() with no args (defaults)
        zed.iParameters();
    end

    % Ensure core defaults even if user overrides broke something
    if ~isfield(zed.pPar, "timeoutSec") || isempty(zed.pPar.timeoutSec)
        zed.pPar.timeoutSec = 1.0;
    end
    if ~isfield(zed.pPar, "streamHz") || ~isstruct(zed.pPar.streamHz)
        zed.pPar.streamHz = struct("image", 15, "depth", 15, "imu", 100, ...
                                   "pose", 15, "pcd", 1, "calib", 0);
    end

    % ---------------------------------------------------------------------
    % Parse args
    % ---------------------------------------------------------------------
    p = inputParser;
    addParameter(p, "UseCallbacks", true, @(x) islogical(x) && isscalar(x));
    addParameter(p, "Include", string.empty, @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});

    useCallbacks = logical(p.Results.UseCallbacks);
    include = string(p.Results.Include);

    % Persist behavior choice for getSensorData() scheduling
    zed.pPar.UseCallbacks = useCallbacks;

    % --------------------------- Node ------------------------------------
    % Your project may already manage ROS2 init elsewhere.
    % Keep it minimal and idempotent.
    try
        if ~isfield(zed.pCom, "node") || isempty(zed.pCom.node)
            zed.pCom.node = ros2node("zed2i_matlab");
        end
    catch excp
        zed.pFlag.Connected = false;
        zed.pFlag.LastError = "ROS2 node creation failed: " + string(excp.message);
        rethrow(excp);
    end

    % --------------------------- Decide streams to subscribe -------------
    streams = resolveStreamsToSubscribe(zed, include);

    % --------------------------- Cleanup old subscribers ------------------
    safeClearSubscriber(zed, "subImage");
    safeClearSubscriber(zed, "subDepth");
    safeClearSubscriber(zed, "subImu");
    safeClearSubscriber(zed, "subOdom");
    safeClearSubscriber(zed, "subPointCloud");
    safeClearSubscriber(zed, "subInfo");

    % Clear cached msgs
    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];
    zed.pCom.lastMsgImu = [];
    zed.pCom.lastMsgOdom = [];
    zed.pCom.lastMsgPointCloud = [];
    zed.pCom.lastMsgInfo = [];

    % Reset flags
    zed.pFlag.HasImage = false;
    zed.pFlag.HasDepth = false;
    zed.pFlag.HasImu = false;
    zed.pFlag.HasPose = false;
    zed.pFlag.HasPointCloud = false;
    zed.pFlag.HasCalibration = false;

    % --------------------------- Create subscribers -----------------------
    try
        qos = zed.sDefaultQosBestEffort();

        if any(streams == "image")
            zed.pCom.subImage = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicImage), ...
                "sensor_msgs/Image", ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subImage.NewMessageFcn = @(~, msg) onImageMsg(zed, msg);
            end
        end

        if any(streams == "depth")
            zed.pCom.subDepth = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicDepth), ...
                "sensor_msgs/Image", ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subDepth.NewMessageFcn = @(~, msg) onDepthMsg(zed, msg);
            end
        end

        if any(streams == "imu")
            zed.pCom.subImu = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicImu), ...
                "sensor_msgs/Imu", ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subImu.NewMessageFcn = @(~, msg) onImuMsg(zed, msg);
            end
        end

        if any(streams == "pose")
            zed.pCom.subOdom = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicOdom), ...
                "nav_msgs/Odometry", ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subOdom.NewMessageFcn = @(~, msg) onOdomMsg(zed, msg);
            end
        end

        if any(streams == "pcd")
            zed.pCom.subPointCloud = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicPointCloud), ...
                "sensor_msgs/PointCloud2", ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subPointCloud.NewMessageFcn = @(~, msg) onPointCloudMsg(zed, msg);
            end
        end

        if any(streams == "calib")
            zed.pCom.subInfo = ros2subscriber( ...
                zed.pCom.node, ...
                string(zed.pPar.topicCameraInfo), ...
                "sensor_msgs/CameraInfo", ...
                "Reliability", "reliable", ...
                "Durability", qos.Durability);

            if useCallbacks
                zed.pCom.subInfo.NewMessageFcn = @(~, msg) onCameraInfoMsg(zed, msg);
            end
        end

        zed.pFlag.Connected = true;
        zed.pFlag.LastError = "";

    catch excp
        zed.pFlag.Connected = false;
        zed.pFlag.LastError = "Subscriber creation failed: " + string(excp.message);
        rethrow(excp);
    end
end

% -------------------------------------------------------------------------
% Stream selection policy
% -------------------------------------------------------------------------

function streams = resolveStreamsToSubscribe(zed, include)
%resolveStreamsToSubscribe Decide which streams to subscribe to.

    allStreams = ["image","depth","imu","pose","pcd","calib"];

    if ~isempty(include)
        streams = intersect(allStreams, include, "stable");
        return;
    end

    streams = strings(0);

    % image/depth always governed by streamHz
    if isStreamEnabledHz(zed, "image")
        streams(end+1) = "image";
    end
    if isStreamEnabledHz(zed, "depth")
        streams(end+1) = "depth";
    end

    % optional sensors are gated by enable flags too
    if isEnabled(zed, "enableImu") && isStreamEnabledHz(zed, "imu")
        streams(end+1) = "imu";
    end

    if isEnabled(zed, "enablePose") && isStreamEnabledHz(zed, "pose")
        streams(end+1) = "pose";
    end

    if isEnabled(zed, "enablePointCloud") && isStreamEnabledHz(zed, "pcd")
        streams(end+1) = "pcd";
    end

    % calib is rare: subscribe only if user wants it (hz>0) OR if not present
    % If you want calib always available, set streamHz.calib = Inf in iParameters.
    if isStreamEnabledHz(zed, "calib")
        streams(end+1) = "calib";
    end

    streams = unique(streams, "stable");
end

function tf = isStreamEnabledHz(zed, name)
%isStreamEnabledHz True if streamHz.(name) exists and is > 0 OR is Inf.

    tf = false;
    if ~isfield(zed.pPar, "streamHz") || ~isstruct(zed.pPar.streamHz)
        return;
    end
    if ~isfield(zed.pPar.streamHz, name)
        return;
    end

    hz = double(zed.pPar.streamHz.(name));
    tf = isinf(hz) || (hz > 0);
end

function tf = isEnabled(zed, fieldName)
%isEnabled True if zed.pPar.(fieldName) exists and is true.

    tf = isfield(zed.pPar, fieldName) && logical(zed.pPar.(fieldName));
end

% -------------------------------------------------------------------------
% Subscriber cleanup
% -------------------------------------------------------------------------

function safeClearSubscriber(zed, fieldName)
%safeClearSubscriber Clear subscriber handle if it exists.
    if isfield(zed.pCom, fieldName) && ~isempty(zed.pCom.(fieldName))
        try
            zed.pCom.(fieldName) = [];
        catch
            % ignore
        end
    end
end

% -------------------------------------------------------------------------
% Callbacks (cache only; no decoding)
% -------------------------------------------------------------------------

function onImageMsg(zed, msg)
    zed.pCom.lastMsgImage = msg;
    zed.pFlag.HasImage = true;
end

function onDepthMsg(zed, msg)
    zed.pCom.lastMsgDepth = msg;
    zed.pFlag.HasDepth = true;
end

function onImuMsg(zed, msg)
    zed.pCom.lastMsgImu = msg;
    zed.pFlag.HasImu = true;
end

function onOdomMsg(zed, msg)
    zed.pCom.lastMsgOdom = msg;
    zed.pFlag.HasPose = true;
end

function onPointCloudMsg(zed, msg)
    zed.pCom.lastMsgPointCloud = msg;
    zed.pFlag.HasPointCloud = true;
end

function onCameraInfoMsg(zed, msg)
    zed.pCom.lastMsgInfo = msg;
    zed.pFlag.HasCalibration = true;
end

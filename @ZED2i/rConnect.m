function rConnect(zed)
%rConnect Create ROS2 node and image/depth subscribers.

    zed.pFlag.LastError = '';

    if zed.pFlag.Connected
        return;
    end

    try
        zed.pCom.node = ros2node(zed.pPar.nodeName);

        zed.pCom.subImage = ros2subscriber( ...
            zed.pCom.node, ...
            zed.pPar.topicImage, ...
            "sensor_msgs/Image");

        zed.pCom.subDepth = ros2subscriber( ...      % NEW
            zed.pCom.node, ...
            zed.pPar.topicDepth, ...
            "sensor_msgs/Image");

        zed.pFlag.Connected = true;
        zed.pFlag.HasImage = false;
        zed.pFlag.HasDepth = false;                  % NEW

    catch excp
        zed.pFlag.Connected = false;
        zed.pFlag.LastError = excp.message;
        rethrow(excp);
    end
end

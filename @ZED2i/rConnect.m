function rConnect(zed)
%rConnect Create ROS2 node and image subscriber.

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

        zed.pFlag.Connected = true;
        zed.pFlag.HasImage = false;

    catch excp
        zed.pFlag.Connected = false;
        zed.pFlag.LastError = excp.message;
        rethrow(excp);
    end
end

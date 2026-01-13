function rDisconnect(zed)
%rDisconnect Release ROS2 resources.

    zed.pFlag.Connected = false;

    if isfield(zed.pCom, 'subImage') && ~isempty(zed.pCom.subImage)
        clear zed.pCom.subImage;
        zed.pCom.subImage = [];
    end

    if isfield(zed.pCom, 'node') && ~isempty(zed.pCom.node)
        clear zed.pCom.node;
        zed.pCom.node = [];
    end

    zed.pCom.lastMsgImage = [];
    zed.pFlag.HasImage = false;
end

function rDisconnect(zed)
%rDisconnect Release ROS2 resources.

    zed.pFlag.Connected = false;

    if isfield(zed.pCom, 'subImage') && ~isempty(zed.pCom.subImage)
        clear zed.pCom.subImage;
        zed.pCom.subImage = [];
    end

    if isfield(zed.pCom, 'subDepth') && ~isempty(zed.pCom.subDepth)   % NEW
        clear zed.pCom.subDepth;
        zed.pCom.subDepth = [];
    end

    if isfield(zed.pCom, 'node') && ~isempty(zed.pCom.node)
        clear zed.pCom.node;
        zed.pCom.node = [];
    end

    if isfield(zed.pCom, 'subInfo') && ~isempty(zed.pCom.subInfo)
        clear zed.pCom.subInfo;
        zed.pCom.subInfo = [];
    end

    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];
    zed.pFlag.HasImage = false;
    zed.pFlag.HasDepth = false;
    zed.pCom.lastMsgInfo = [];
    zed.pFlag.HasCalibration = false;
end

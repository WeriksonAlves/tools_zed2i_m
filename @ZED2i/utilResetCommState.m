function utilResetCommState(zed)
%utilResetCommState Clear ROS2 node, subscribers and last received messages.
%
% This is the single source of truth for zeroing pCom to a known baseline.
% It is safe to call multiple times (idempotent).

    if isfield(zed, "pCom") && ~isempty(zed.pCom)
        % Subscribers
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
        if isfield(zed.pCom, "subPointCloud") && ~isempty(zed.pCom.subPointCloud)
            clear zed.pCom.subPointCloud;
        end

        % Node
        if isfield(zed.pCom, "node") && ~isempty(zed.pCom.node)
            clear zed.pCom.node;
        end
    end

    % Zera comunicação em um estado conhecido
    zed.pCom.node              = [];
    zed.pCom.subImage          = [];
    zed.pCom.subDepth          = [];
    zed.pCom.subInfo           = [];
    zed.pCom.lastMsgImage      = [];
    zed.pCom.lastMsgDepth      = [];
    zed.pCom.lastMsgInfo       = [];
    zed.pCom.subImu            = [];
    zed.pCom.lastMsgImu        = [];
    zed.pCom.subOdom           = [];
    zed.pCom.lastMsgOdom       = [];
    zed.pCom.subPointCloud     = [];
    zed.pCom.lastMsgPointCloud = [];
end

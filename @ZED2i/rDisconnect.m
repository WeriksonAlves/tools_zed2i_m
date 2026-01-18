function rDisconnect(zed)
%rDisconnect Release ROS2 resources and reset communication state.
%
% This method is idempotent: it can be called multiple times safely.

    % Mesmo se já estiver desconectado, fazemos um cleanup defensivo
    clearCommState(zed);

    % Flags em estado desconectado conhecido
    zed.pFlag.Connected      = false;
    zed.pFlag.HasImage       = false;
    zed.pFlag.HasDepth       = false;
    if isfield(zed.pFlag, "HasCalibration")
        zed.pFlag.HasCalibration = false;
    end
    if isfield(zed.pFlag, "HasImu")
        zed.pFlag.HasImu = false;
    end

end

% -------------------------------------------------------------------------
% Local helper
% -------------------------------------------------------------------------
function clearCommState(zed)
%clearCommState Clear node, subscribers and last received messages.

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

        % Node
        if isfield(zed.pCom, "node") && ~isempty(zed.pCom.node)
            clear zed.pCom.node;
        end
    end

    % Zera comunicação em um estado conhecido
    zed.pCom.node         = [];
    zed.pCom.subImage     = [];
    zed.pCom.subDepth     = [];
    zed.pCom.subInfo      = [];
    zed.pCom.lastMsgImage = [];
    zed.pCom.lastMsgDepth = [];
    zed.pCom.lastMsgInfo  = [];
    zed.pCom.subImu       = [];
    zed.pCom.lastMsgImu   = [];

end

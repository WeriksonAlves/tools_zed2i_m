function sResetCommState(zed)
%sResetCommState Reset ROS2 handles and last received messages (idempotent).

    % Ensure pCom exists
    if ~isfield(zed, "pCom") || isempty(zed.pCom)
        zed.pCom = struct();
    end

    % List all comm fields (single source of truth)
    commFields = { ...
        "node", ...
        "subImage", "subDepth", "subInfo", ...
        "subImu", "subOdom", "subPointCloud", ...
        "lastMsgImage", "lastMsgDepth", "lastMsgInfo", ...
        "lastMsgImu", "lastMsgOdom", "lastMsgPointCloud" ...
    };

    % Drop references (avoid `clear`, keep idempotent)
    for i = 1:numel(commFields)
        f = commFields{i};
        if isfield(zed.pCom, f)
            zed.pCom.(f) = [];
        else
            % Pre-create missing fields to stabilize the struct shape
            zed.pCom.(f) = [];
        end
    end
end

function sResetCommState(zed)
%sResetCommState Reset ROS2 handles and last received messages.
%
% This method is intentionally idempotent. It can be called multiple times
% without throwing errors.
%
% Important:
% - Subscriber callbacks are explicitly disabled before dropping references.
% - ROS2 subscriber/node handles are released defensively.
% - Struct fields are kept stable after cleanup.

    % ---------------------------------------------------------------------
    % Ensure communication struct exists
    % ---------------------------------------------------------------------
    if isempty(zed.pCom) || ~isstruct(zed.pCom)
        zed.pCom = struct();
    end

    % ---------------------------------------------------------------------
    % Disable callbacks before releasing subscriber handles.
    %
    % This is important because callbacks may keep references to the ZED2i
    % object and delay subscriber cleanup.
    % ---------------------------------------------------------------------
    subscriberFields = { ...
        "subImage", ...
        "subDepth", ...
        "subInfo", ...
        "subImu", ...
        "subOdom", ...
        "subPointCloud" ...
    };

    for i = 1:numel(subscriberFields)
        fieldName = subscriberFields{i};

        if isfield(zed.pCom, fieldName) && ~isempty(zed.pCom.(fieldName))
            safeDisableCallback_(zed.pCom.(fieldName));
        end
    end

    % ---------------------------------------------------------------------
    % Release subscriber handles before releasing the node.
    % ---------------------------------------------------------------------
    for i = 1:numel(subscriberFields)
        fieldName = subscriberFields{i};

        if isfield(zed.pCom, fieldName) && ~isempty(zed.pCom.(fieldName))
            safeReleaseRosHandle_(zed.pCom.(fieldName));
        end

        zed.pCom.(fieldName) = [];
    end

    % ---------------------------------------------------------------------
    % Release node handle last.
    % ---------------------------------------------------------------------
    if isfield(zed.pCom, "node") && ~isempty(zed.pCom.node)
        safeReleaseRosHandle_(zed.pCom.node);
    end
    zed.pCom.node = [];

    % ---------------------------------------------------------------------
    % Reset cached messages.
    % ---------------------------------------------------------------------
    lastMsgFields = { ...
        "lastMsgImage", ...
        "lastMsgDepth", ...
        "lastMsgInfo", ...
        "lastMsgImu", ...
        "lastMsgOdom", ...
        "lastMsgPointCloud" ...
    };

    for i = 1:numel(lastMsgFields)
        fieldName = lastMsgFields{i};
        zed.pCom.(fieldName) = [];
    end
end

% -------------------------------------------------------------------------
% Local helper functions
% -------------------------------------------------------------------------
function safeDisableCallback_(rosHandle)
%safeDisableCallback_ Disable ROS2 callback if the handle supports it.

    try
        if isobject(rosHandle) && isprop(rosHandle, "NewMessageFcn")
            rosHandle.NewMessageFcn = [];
        end
    catch
        % Best-effort cleanup. Ignore callback cleanup errors.
    end
end

function safeReleaseRosHandle_(rosHandle)
%safeReleaseRosHandle_ Try to explicitly release a ROS2 object.

    try
        if isobject(rosHandle)
            delete(rosHandle);
        end
    catch
        % Some MATLAB ROS2 objects may not expose delete().
        % Dropping the reference is still useful.
    end
end
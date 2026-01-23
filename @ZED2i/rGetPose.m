function pose = rGetPose(zed)
%rGetPose Get pose/odometry data from ROS2 nav_msgs/Odometry (optional feature).
%
% Returns a struct with stable fields:
%   Timestamp (datetime)
%   FrameId (string)
%   ChildFrameId (string)
%   Position (1x3) [x y z] (m)
%   OrientationQuat (1x4) [w x y z]
%   LinearVelocity (1x3) (m/s)
%   AngularVelocity (1x3) (rad/s)
%   PoseCovariance (6x6)
%   TwistCovariance (6x6)

    % Não alocamos struct completo aqui; só em caso de erro ou no parse.
    pose = struct();

    % ---------------------------------------------------------------------
    % Sanity checks
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        zed.pFlag.HasPose = false;
        pose = buildEmptyPoseStruct();
        return;
    end

    if ~isfield(zed.pCom, "subOdom") || isempty(zed.pCom.subOdom)
        zed.pFlag.LastError = ...
            "Pose subscriber not initialized. Enable pose (enablePose=true) before rConnect().";
        zed.pFlag.HasPose = false;
        pose = buildEmptyPoseStruct();
        return;
    end

    % ---------------------------------------------------------------------
    % Try to receive a fresh message (with cached fallback)
    % ---------------------------------------------------------------------
    [msg, ok, lastErr] = receiveWithCache( ...
        zed.pCom.subOdom, ...
        getFieldOrEmpty(zed.pCom, "lastMsgOdom"), ...
        zed.pPar.timeoutSec, ...
        "Odom");

    if ~ok
        zed.pFlag.LastError = lastErr;
        zed.pFlag.HasPose = false;
        pose = buildEmptyPoseStruct();
        return;
    end

    zed.pCom.lastMsgOdom = msg;

    % ---------------------------------------------------------------------
    % Parse Odom message
    % ---------------------------------------------------------------------
    try
        pose = parseOdomMessage(msg);
        zed.pData.Pose = pose;
        zed.pFlag.HasPose = true;
        zed.pFlag.LastError = "";
    catch excp
        zed.pFlag.LastError = "Odom parse failed: " + excp.message;
        zed.pFlag.HasPose = false;
        pose = buildEmptyPoseStruct();
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function pose = buildEmptyPoseStruct()
    pose = struct( ...
        "Timestamp", datetime('now'), ...
        "FrameId", "", ...
        "ChildFrameId", "", ...
        "Position", [NaN NaN NaN], ...
        "OrientationQuat", [NaN NaN NaN NaN], ...
        "LinearVelocity", [NaN NaN NaN], ...
        "AngularVelocity", [NaN NaN NaN], ...
        "PoseCovariance", NaN(6, 6), ...
        "TwistCovariance", NaN(6, 6) ...
    );
end

function flag = safeFlag(flags, fieldName)
    flag = false;
    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

function value = getFieldOrEmpty(s, fieldName)
%getFieldOrEmpty Retrieve struct field or [] if not present.
    if isstruct(s) && isfield(s, fieldName)
        value = s.(fieldName);
    else
        value = [];
    end
end

function [msgOut, ok, errMsg] = receiveWithCache(sub, lastMsg, timeoutSec, label)
%receiveWithCache Try to receive a fresh ROS2 message with cached fallback.

    msgOut = [];
    ok = false;
    errMsg = "";

    try
        msgOut = receive(sub, timeoutSec);
        ok = true;
        return;
    catch excp
        if ~isempty(lastMsg)
            msgOut = lastMsg;
            ok = true;
            errMsg = "";
        else
            errMsg = label + " receive failed: " + excp.message;
        end
    end
end

function pose = parseOdomMessage(msg)
%parseOdomMessage Decode nav_msgs/Odometry into a stable struct.

    % Timestamp
    ts = datetime('now');

    frameId = "";
    childFrameId = "";

    if isfield(msg, "header")
        if isfield(msg.header, "frame_id")
            frameId = string(msg.header.frame_id);
        end
        % Poderíamos converter header.stamp aqui se quisermos mais precisão.
    end
    if isfield(msg, "child_frame_id")
        childFrameId = string(msg.child_frame_id);
    end

    % Pose
    p = msg.pose.pose.position;
    q = msg.pose.pose.orientation; % ROS: x,y,z,w

    position = [double(p.x), double(p.y), double(p.z)];
    orientationQuat = [double(q.w), double(q.x), double(q.y), double(q.z)];

    % Twist
    v = msg.twist.twist.linear;
    w = msg.twist.twist.angular;

    linVel = [double(v.x), double(v.y), double(v.z)];
    angVel = [double(w.x), double(w.y), double(w.z)];

    % Covariances (flattened arrays)
    poseCov = NaN(6, 6);
    twistCov = NaN(6, 6);

    if isfield(msg.pose, "covariance")
        poseCov = reshape(double(msg.pose.covariance), [6, 6])';
    end
    if isfield(msg.twist, "covariance")
        twistCov = reshape(double(msg.twist.covariance), [6, 6])';
    end

    pose = struct( ...
        "Timestamp", ts, ...
        "FrameId", frameId, ...
        "ChildFrameId", childFrameId, ...
        "Position", position, ...
        "OrientationQuat", orientationQuat, ...
        "LinearVelocity", linVel, ...
        "AngularVelocity", angVel, ...
        "PoseCovariance", poseCov, ...
        "TwistCovariance", twistCov ...
    );
end

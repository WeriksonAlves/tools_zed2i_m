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

    pose = buildEmptyPoseStruct();

    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        return;
    end

    if ~isfield(zed.pCom, "subOdom") || isempty(zed.pCom.subOdom)
        zed.pFlag.LastError = ...
            "Pose subscriber not initialized. Enable pose (pPar.enablePose=true) before rConnect().";
        return;
    end

    msg = [];
    hasNewMsg = false;

    % ---------------------------------------------------------------------
    % Always try to receive a fresh message
    % ---------------------------------------------------------------------
    try
        msg = receive(zed.pCom.subOdom, zed.pPar.timeoutSec);
        zed.pCom.lastMsgOdom = msg;
        hasNewMsg = true;
    catch excp
        % On timeout or receive failure, fall back to last valid message
        if isfield(zed.pCom, "lastMsgOdom") && ~isempty(zed.pCom.lastMsgOdom)
            msg = zed.pCom.lastMsgOdom;
        else
            zed.pFlag.LastError = "Odom receive failed: " + excp.message;
            zed.pFlag.HasPose = false;
            return;
        end
    end

    % ---------------------------------------------------------------------
    % Parse Odom message (new or cached)
    % ---------------------------------------------------------------------
    try
        pose = parseOdomMessage(msg);

        % Cache decoded snapshot
        zed.pData.Pose = pose;
        zed.pFlag.HasPose = true;

        if hasNewMsg
            zed.pFlag.LastError = "";
        end

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

function pose = parseOdomMessage(msg)
    pose = buildEmptyPoseStruct();
    pose.Timestamp = datetime('now');

    % Header and frames
    if isfield(msg, "header") && isfield(msg.header, "frame_id")
        pose.FrameId = string(msg.header.frame_id);
    end
    if isfield(msg, "child_frame_id")
        pose.ChildFrameId = string(msg.child_frame_id);
    end

    % Pose
    p = msg.pose.pose.position;
    q = msg.pose.pose.orientation; % ROS: x,y,z,w

    pose.Position = [double(p.x), double(p.y), double(p.z)];
    pose.OrientationQuat = [double(q.w), double(q.x), double(q.y), double(q.z)];

    % Twist
    v = msg.twist.twist.linear;
    w = msg.twist.twist.angular;

    pose.LinearVelocity = [double(v.x), double(v.y), double(v.z)];
    pose.AngularVelocity = [double(w.x), double(w.y), double(w.z)];

    % Covariances (flattened arrays)
    if isfield(msg.pose, "covariance")
        pose.PoseCovariance = reshape(double(msg.pose.covariance), [6, 6])';
    end
    if isfield(msg.twist, "covariance")
        pose.TwistCovariance = reshape(double(msg.twist.covariance), [6, 6])';
    end
end

function [pose, ok, err] = rosGetPose(zed, varargin)
%rosGetPose Decode odometry from cached or freshly received message.
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    pose = struct();
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subOdom") || isempty(zed.pCom.subOdom)
        err = "Pose subscriber not initialized (subOdom).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subOdom, zed.pPar.timeoutSec);
            zed.pCom.lastMsgOdom = msg;
        catch excp
            err = "Odom receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgOdom")
            msg = zed.pCom.lastMsgOdom;
        end
        if isempty(msg)
            err = "Odom cache empty (no lastMsgOdom).";
            return;
        end
    end

    try
        pose = parseOdomMessage(msg);
        ok = true;
    catch excp
        err = "Odom decode failed: " + string(excp.message);
    end
end

function pose = parseOdomMessage(msg)
    pose = struct( ...
        "Timestamp", datetime('now'), ...
        "FrameId", "", ...
        "ChildFrameId", "", ...
        "Position", [NaN NaN NaN], ...
        "OrientationEuler", [NaN NaN NaN], ...
        "LinearVelocity", [NaN NaN NaN], ...
        "AngularVelocity", [NaN NaN NaN], ...
        "PoseCovariance", NaN(6, 6), ...
        "TwistCovariance", NaN(6, 6) ...
    );

    if isfield(msg, "header") && isfield(msg.header, "frame_id")
        pose.FrameId = string(msg.header.frame_id);
    end
    if isfield(msg, "child_frame_id")
        pose.ChildFrameId = string(msg.child_frame_id);
    end

    p = msg.pose.pose.position;
    q = msg.pose.pose.orientation;

    pose.Position = [double(p.x), double(p.y), double(p.z)];

    qwxyz = [double(q.w), double(q.x), double(q.y), double(q.z)];
    pose.OrientationEuler = utilQuatToEulerRad(qwxyz);

    if isfield(msg.pose, "covariance")
        pose.PoseCovariance = reshape(double(msg.pose.covariance), [6, 6])';
    end
    if isfield(msg.twist, "covariance")
        pose.TwistCovariance = reshape(double(msg.twist.covariance), [6, 6])';
    end
end

function eul = utilQuatToEulerRad(q)
    if ~ismatrix(q) || size(q, 2) ~= 4
        error("utilQuatToEulerRad:InvalidSize", ...
            "Input q must be of size Nx4 with format [w x y z].");
    end

    q = double(q);
    w = q(:, 1); x = q(:, 2); y = q(:, 3); z = q(:, 4);

    sinr_cosp = 2 .* (w .* x + y .* z);
    cosr_cosp = 1 - 2 .* (x.^2 + y.^2);
    roll = atan2(sinr_cosp, cosr_cosp);

    sinp = 2 .* (w .* y - z .* x);
    sinp = max(min(sinp, 1), -1);
    pitch = asin(sinp);

    siny_cosp = 2 .* (w .* z + x .* y);
    cosy_cosp = 1 - 2 .* (y.^2 + z.^2);
    yaw = atan2(siny_cosp, cosy_cosp);

    eul = [roll, pitch, yaw];
end

function [imu, ok, err] = rosGetImu(zed, varargin)
%rosGetImu Decode IMU from cached or freshly received message.
%
% Name-Value:
%   Mode ("cached"|"fetch") : default "cached"

    p = inputParser;
    addParameter(p, "Mode", "cached", @(x) isstring(x) || ischar(x));
    parse(p, varargin{:});
    mode = string(p.Results.Mode);

    imu = struct();
    ok = false;
    err = "";

    if ~isfield(zed.pCom, "subImu") || isempty(zed.pCom.subImu)
        err = "IMU subscriber not initialized (subImu).";
        return;
    end

    msg = [];
    if mode == "fetch"
        try
            msg = receive(zed.pCom.subImu, zed.pPar.timeoutSec);
            zed.pCom.lastMsgImu = msg;
        catch excp
            err = "IMU receive failed: " + string(excp.message);
            return;
        end
    else
        if isfield(zed.pCom, "lastMsgImu")
            msg = zed.pCom.lastMsgImu;
        end
        if isempty(msg)
            err = "IMU cache empty (no lastMsgImu).";
            return;
        end
    end

    try
        imu = parseImuMessage(msg);
        ok = true;
    catch excp
        err = "IMU decode failed: " + string(excp.message);
    end
end

function imu = parseImuMessage(msg)
    imu = struct( ...
        "TimestampSec", 0.0, ...
        "OrientationQuat", [NaN NaN NaN NaN], ...
        "AngularVelocity", [NaN NaN NaN], ...
        "LinearAcceleration", [NaN NaN NaN], ...
        "OrientationCovariance", NaN(3, 3), ...
        "AngularVelocityCovariance", NaN(3, 3), ...
        "LinearAccelerationCovariance", NaN(3, 3) ...
    );

    qx = msg.orientation.x;
    qy = msg.orientation.y;
    qz = msg.orientation.z;
    qw = msg.orientation.w;

    imu.OrientationQuat = [double(qw) double(qx) double(qy) double(qz)];

    imu.AngularVelocity = [ ...
        double(msg.angular_velocity.x), ...
        double(msg.angular_velocity.y), ...
        double(msg.angular_velocity.z) ...
    ];

    imu.LinearAcceleration = [ ...
        double(msg.linear_acceleration.x), ...
        double(msg.linear_acceleration.y), ...
        double(msg.linear_acceleration.z) ...
    ];

    imu.OrientationCovariance = reshape(double(msg.orientation_covariance), [3, 3])';
    imu.AngularVelocityCovariance = reshape(double(msg.angular_velocity_covariance), [3, 3])';
    imu.LinearAccelerationCovariance = reshape(double(msg.linear_acceleration_covariance), [3, 3])';
end

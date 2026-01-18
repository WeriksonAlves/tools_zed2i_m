function imu = rGetImu(zed)
%rGetImu Get IMU data from ROS2 sensor_msgs/Imu (optional feature).
%
% Returns a struct with stable fields:
%   Timestamp (datetime)
%   OrientationQuat  (1x4) [w x y z]
%   AngularVelocity  (1x3) [wx wy wz] rad/s
%   LinearAcceleration (1x3) [ax ay az] m/s^2
%   OrientationCovariance (3x3)
%   AngularVelocityCovariance (3x3)
%   LinearAccelerationCovariance (3x3)

    imu = buildEmptyImuStruct();

    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call rConnect() first.";
        return;
    end

    if ~isfield(zed.pCom, "subImu") || isempty(zed.pCom.subImu)
        zed.pFlag.LastError = ...
            "IMU subscriber not initialized. Enable IMU (pPar.enableImu=true) before rConnect().";
        return;
    end

    msg = [];
    hasNewMsg = false;

    % ---------------------------------------------------------------------
    % Always try to read a new message
    % ---------------------------------------------------------------------
    try
        msg = receive(zed.pCom.subImu, zed.pPar.timeoutSec);
        zed.pCom.lastMsgImu = msg;
        hasNewMsg = true;
    catch excp
        % If timeout/error, try to reuse last valid message
        if isfield(zed.pCom, "lastMsgImu") && ~isempty(zed.pCom.lastMsgImu)
            msg = zed.pCom.lastMsgImu;
        else
            zed.pFlag.LastError = "IMU receive failed: " + excp.message;
            zed.pFlag.HasImu = false;
            return;
        end
    end

    % ---------------------------------------------------------------------
    % Parse the message (new or reused)
    % ---------------------------------------------------------------------
    try
        imu = parseImuMessage(msg);

        % Cache decoded snapshot
        zed.pData.Imu = imu;
        zed.pFlag.HasImu = true;

    catch excp
        zed.pFlag.LastError = "IMU parse failed: " + excp.message;
        zed.pFlag.HasImu = false;
        imu = buildEmptyImuStruct();
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------
function imu = buildEmptyImuStruct()
    imu = struct( ...
        "Timestamp", datetime('now'), ...
        "OrientationQuat", [NaN NaN NaN NaN], ...
        "AngularVelocity", [NaN NaN NaN], ...
        "LinearAcceleration", [NaN NaN NaN], ...
        "OrientationCovariance", NaN(3, 3), ...
        "AngularVelocityCovariance", NaN(3, 3), ...
        "LinearAccelerationCovariance", NaN(3, 3) ...
    );
end

function flag = safeFlag(flags, fieldName)
    flag = false;
    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

function imu = parseImuMessage(msg)
    imu = buildEmptyImuStruct();

    imu.Timestamp = datetime('now');

    % Quaternion in ROS: x,y,z,w -> store as [w x y z]
    qx = double(msg.orientation.x);
    qy = double(msg.orientation.y);
    qz = double(msg.orientation.z);
    qw = double(msg.orientation.w);
    imu.OrientationQuat = [qw qx qy qz];

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

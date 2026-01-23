function imu = rGetImu(zed)
%rGetImu Get IMU data from ROS2 sensor_msgs/Imu (optional feature).
%
% Returns a struct with stable fields:
%   Timestamp                 (datetime)
%   OrientationQuat           (1x4) [w x y z]
%   AngularVelocity           (1x3) [wx wy wz] rad/s
%   LinearAcceleration        (1x3) [ax ay az] m/s^2
%   OrientationCovariance     (3x3)
%   AngularVelocityCovariance (3x3)
%   LinearAccelerationCovariance (3x3)

    % Não alocamos struct completo aqui.
    % Só criamos o "empty" em caso de erro, para evitar trabalho inútil.
    imu = struct();

    % ---------------------------------------------------------------------
    % Sanity checks
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call lcConnect() first.";
        imu = buildEmptyImuStruct();
        zed.pFlag.HasImu = false;
        return;
    end

    if ~isfield(zed.pCom, "subImu") || isempty(zed.pCom.subImu)
        zed.pFlag.LastError = ...
            "IMU subscriber not initialized. Enable IMU (enableImu=true) before lcConnect().";
        imu = buildEmptyImuStruct();
        zed.pFlag.HasImu = false;
        return;
    end

    % ---------------------------------------------------------------------
    % Try to receive a fresh message (with cached fallback)
    % ---------------------------------------------------------------------
    [msg, ok, lastErr] = receiveWithCache( ...
        zed.pCom.subImu, ...
        getFieldOrEmpty(zed.pCom, "lastMsgImu"), ...
        zed.pPar.timeoutSec, ...
        "IMU");

    if ~ok
        zed.pFlag.LastError = lastErr;
        zed.pFlag.HasImu = false;
        imu = buildEmptyImuStruct();
        return;
    end

    zed.pCom.lastMsgImu = msg;

    % ---------------------------------------------------------------------
    % Parse IMU message
    % ---------------------------------------------------------------------
    try
        imu = parseImuMessage(msg);
        zed.pData.Imu = imu;
        zed.pFlag.HasImu = true;
        zed.pFlag.LastError = "";
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
%
%   [msgOut, ok, errMsg] = receiveWithCache(sub, lastMsg, timeoutSec, label)
%
%   ok      : true if either a new message or a cached one is returned
%   errMsg  : non-empty only when no message is available

    msgOut = [];
    ok = false;
    errMsg = "";

    try
        msgOut = receive(sub, timeoutSec);
        ok = true;
        return;
    catch excp
        % On timeout or error, fall back to last valid message
        if ~isempty(lastMsg)
            msgOut = lastMsg;
            ok = true;
            errMsg = "";
        else
            errMsg = label + " receive failed: " + excp.message;
        end
    end
end

function imu = parseImuMessage(msg)
%parseImuMessage Decode sensor_msgs/Imu into a stable struct.

    imu = struct( ...
        "Timestamp", datetime('now'), ...
        "OrientationQuat", [NaN NaN NaN NaN], ...
        "AngularVelocity", [NaN NaN NaN], ...
        "LinearAcceleration", [NaN NaN NaN], ...
        "OrientationCovariance", NaN(3, 3), ...
        "AngularVelocityCovariance", NaN(3, 3), ...
        "LinearAccelerationCovariance", NaN(3, 3) ...
    );

    % Timestamp: se quisermos, podemos usar header.stamp.*; por enquanto, now().
    if isfield(msg, "header") && isfield(msg.header, "stamp")
        % Aqui poderia ser feita conversão precisa do stamp; mantemos 'now' simplificado.
        imu.Timestamp = datetime('now');
    else
        imu.Timestamp = datetime('now');
    end

    % Quaternion in ROS: x,y,z,w -> store as [w x y z]
    qx = msg.orientation.x;
    qy = msg.orientation.y;
    qz = msg.orientation.z;
    qw = msg.orientation.w;
    imu.OrientationQuat = [qw qx qy qz];

    imu.AngularVelocity = [ ...
        msg.angular_velocity.x, ...
        msg.angular_velocity.y, ...
        msg.angular_velocity.z ...
    ];

    imu.LinearAcceleration = [ ...
        msg.linear_acceleration.x, ...
        msg.linear_acceleration.y, ...
        msg.linear_acceleration.z ...
    ];

    imu.OrientationCovariance = reshape(double(msg.orientation_covariance), [3, 3])';
    imu.AngularVelocityCovariance = reshape(double(msg.angular_velocity_covariance), [3, 3])';
    imu.LinearAccelerationCovariance = reshape(double(msg.linear_acceleration_covariance), [3, 3])';
end

function rDisconnect(zed)
%rDisconnect Release ROS2 resources and reset communication state.
%
% This method is idempotent: it can be called multiple times safely.

    % Sempre fazemos um cleanup defensivo da comunicação
    zed.mAuxResetCommState();

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
    if isfield(zed.pFlag, "HasPose")
        zed.pFlag.HasPose = false;
    end
    if isfield(zed.pFlag, "HasPointCloud")
        zed.pFlag.HasPointCloud = false;
    end

    % Mantemos LastError como está (útil para debug)
end

function iControlVariables(zed)
% iControlVariables Initialize control/state vectors for posture.
%
% Notes:
% - Single assignment reduces repeated handle writes.
% - Keeps dimensions unchanged to preserve behavior.

    pPos = struct();

    pPos.X    = zeros(12, 1);  % Real Pose
    pPos.Xa   = zeros(12, 1);  % Previous Pose
    pPos.Xo   = zeros(12, 1);  % Bias pose: Calibration

    pPos.Xd   = zeros(12, 1);  % Desired Pose
    pPos.Xda  = zeros(12, 1);  % Previous Desired Pose
    pPos.dXd  = zeros(12, 1);  % Desired first derivative Pose

    pPos.Xtil = zeros(12, 1);  % Posture Error
    pPos.dX   = zeros(12, 1);  % First derivative

    zed.pPos = pPos;
end

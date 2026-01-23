function img = getImage(zed)
%getImage Blocking receive + decode of the RGB image stream.
%
%   img = zed.getImage()
%
% img : last RGB frame (uint8), or [] if unavailable

    img = [];

    % ---------------------------------------------------------------------
    % Sanity checks
    % ---------------------------------------------------------------------
    if ~safeFlag(zed.pFlag, "Connected")
        zed.pFlag.LastError = "Not connected. Call lcConnect() first.";
        zed.pFlag.HasImage = false;
        return;
    end

    if ~isfield(zed.pCom, "subImage") || isempty(zed.pCom.subImage)
        zed.pFlag.LastError = ...
            "Image subscriber not initialized. Check topics and lcConnect().";
        zed.pFlag.HasImage = false;
        return;
    end

    % ---------------------------------------------------------------------
    % Blocking receive + decode
    % ---------------------------------------------------------------------
    try
        msgImg = receive(zed.pCom.subImage, zed.pPar.timeoutSec);
        zed.pCom.lastMsgImage = msgImg;

        img = rosReadImage(msgImg);

        zed.pData.Image     = img;
        zed.pFlag.HasImage  = true;
        zed.pFlag.LastError = "";

        zed.mAuxUpdateFps("image");

    catch excp
        if isfield(zed.pData, "Metrics") && isfield(zed.pData.Metrics, "ImageDrops")
            zed.pData.Metrics.ImageDrops = zed.pData.Metrics.ImageDrops + 1;
        end

        zed.pFlag.HasImage  = false;
        zed.pFlag.LastError = "Image receive failed: " + string(excp.message);
        img = [];
    end
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------

function flag = safeFlag(flags, fieldName)
%safeFlag Return logical flag if available, otherwise false.

    flag = false;
    if isstruct(flags) && isfield(flags, fieldName)
        flag = logical(flags.(fieldName));
    end
end

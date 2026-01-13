classdef ZED2i < handle
    %ZED2i Minimal ROS2 wrapper for ZED2i image stream (MATLAB R2025a).
    %
    % Minimal API (lab-style):
    %   - rConnect()
    %   - rGrab()
    %   - rGetImage()
    %   - rDisconnect()
    %
    % Internal organization:
    %   pPar  : parameters
    %   pFlag : flags / health
    %   pData : decoded buffers + metrics
    %   pCom  : ROS2 handles + last messages

    properties
        pID
        pPar
        pFlag
        pData
        pCom
    end

    methods
        function obj = ZED2i(id)
            if nargin < 1
                id = 0;
            end

            obj.pID = id;

            obj.pPar = struct();
            obj.pFlag = struct();
            obj.pData = struct();
            obj.pCom = struct();

            obj.iParameters();
            obj.iControlVariables();
        end
    end
end

classdef ZED2i < handle
    %ZED2i Minimal ROS2 wrapper for ZED2i image stream (MATLAB R2025a).
    %
    % Minimal API (lab-style):
    %   - rConnect()
    %   - rGrab()
    %   - rGetImage()
    %   - rGetDepth()
    %   - rGetCalibration()
    %   - rGetSensorData()
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
        function obj = ZED2i(id, varargin)
            %ZED2i Constructor.
            %
            % Usage:
            %   zed = ZED2i();
            %   zed = ZED2i(id);
            %   zed = ZED2i("timeoutSec", 1.0, "fpsAlpha", 0.1);
            %   zed = ZED2i(id, "nodeName", "custom_node");
            %
            % ID is currently stored for lab conventions, but not used
            % internally by the ROS2 integration.

            % -------------------- Handle optional ID --------------------
            if nargin == 0
                obj.pID = 0;
                nameValueArgs = {};
            elseif nargin >= 1 && ~ischar(id) && ~isstring(id)
                % First argument is treated as numeric ID (lab compatibility)
                obj.pID = id;
                nameValueArgs = varargin;
            else
                % No numeric ID provided; shift all args to Name-Value parsing
                obj.pID = 0;
                nameValueArgs = [{id}, varargin];
            end

            % Initialize internal structs
            obj.pPar  = struct();
            obj.pFlag = struct();
            obj.pData = struct();
            obj.pCom  = struct();

            % Default configuration and control variables
            obj.iParameters();
            obj.iControlVariables();

            % Apply Name-Value overrides, if any
            if ~isempty(nameValueArgs)
                applyNameValueOverrides(obj, nameValueArgs{:});
            end
        end
    end
end

% -------------------------------------------------------------------------
% Local helper (constructor scope)
% -------------------------------------------------------------------------
function applyNameValueOverrides(obj, varargin)
%applyNameValueOverrides Override selected pPar fields using Name-Value pairs.
%
% Only recognized fields are applied; others trigger an error to catch
% typos early.

    if mod(numel(varargin), 2) ~= 0
        error("ZED2i:Constructor:InvalidNameValue", ...
            "Name-Value arguments must come in pairs.");
    end

    % String array, não cell:
    validNames = [ ...
        "timeoutSec", ...
        "fpsAlpha", ...
        "nodeName", ...
        "topicImage", ...
        "topicDepth", ...
        "topicCameraInfo" ...
    ];

    for k = 1:2:numel(varargin)
        name  = varargin{k};
        value = varargin{k+1};

        if ~(ischar(name) || isstring(name))
            error("ZED2i:Constructor:InvalidName", ...
                "Name in Name-Value pair must be char or string.");
        end

        nameStr = string(name);

        % Valida nome
        if ~any(nameStr == validNames)
            error("ZED2i:Constructor:UnknownParameter", ...
                "Unknown parameter name '%s'.", nameStr);
        end

        switch nameStr
            case "timeoutSec"
                obj.pPar.timeoutSec = double(value);

            case "fpsAlpha"
                obj.pPar.fpsAlpha = double(value);

            case "nodeName"
                obj.pPar.nodeName = string(value);

            case "topicImage"
                obj.pPar.topicImage = string(value);

            case "topicDepth"
                obj.pPar.topicDepth = string(value);

            case "topicCameraInfo"
                obj.pPar.topicCameraInfo = string(value);
        end
    end
end

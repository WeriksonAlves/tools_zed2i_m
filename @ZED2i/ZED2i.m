classdef ZED2i < handle
    %ZED2i Minimal ROS2 wrapper for ZED2i (MATLAB R2025a).
    %
    % Minimal API (lab-style):
    %   - rConnect()
    %   - rGetImage()
    %   - rGetDepth()
    %   - rGetCalibration()
    %   - rGetSensorData()
    %   - rGetImu()            % opcional, se enableImu = true
    %   - rGetPose()           % opcional, se enablePose = true
    %   - rGetPointCloud()     % opcional, se enablePointCloud = true
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
            % Usage examples:
            %   zed = ZED2i();
            %   zed = ZED2i(1);
            %   zed = ZED2i("timeoutSec", 1.0, "fpsAlpha", 0.1);
            %   zed = ZED2i(1, "nodeName", "custom_node");
            %   zed = ZED2i("enableImu", true, "enablePose", true);
            %
            % ID é mantido por convenção de laboratório, mas não é usado
            % diretamente na integração ROS2.

            % -------------------- Handle optional ID --------------------
            if nargin == 0
                obj.pID = 0;
                nameValueArgs = {};
            elseif nargin >= 1 && ~ischar(id) && ~isstring(id)
                % Primeiro argumento é tratado como ID numérico
                obj.pID = id;
                nameValueArgs = varargin;
            else
                % Sem ID numérico; todos argumentos vão para Name-Value
                obj.pID = 0;
                nameValueArgs = [{id}, varargin];
            end

            % -------------------- Structs internos --------------------
            obj.pPar  = struct();
            obj.pFlag = struct();
            obj.pData = struct();
            obj.pCom  = struct();

            % -------------------- Configuração padrão --------------------
            obj.iParameters();
            obj.iControlVariables();

            % -------------------- Overrides via Name-Value ----------------
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

    % Lista de parâmetros suportados (mantida em sincronia com iParameters)
    validNames = [ ...
        "timeoutSec", ...
        "fpsAlpha", ...
        "fps", ...
        "nodeName", ...
        "topicImage", ...
        "topicImageRight", ...
        "topicDepth", ...
        "topicCameraInfo", ...
        "enableImu", ...
        "topicImu", ...
        "enablePose", ...
        "topicPose", ...
        "topicOdom", ...
        "enablePointCloud", ...
        "topicPointCloud" ...
    ];

    for k = 1:2:numel(varargin)
        name  = varargin{k};
        value = varargin{k+1};

        if ~(ischar(name) || isstring(name))
            error("ZED2i:Constructor:InvalidName", ...
                "Name in Name-Value pair must be char or string.");
        end

        nameStr = string(name);

        if ~any(nameStr == validNames)
            error("ZED2i:Constructor:UnknownParameter", ...
                "Unknown parameter name '%s'.", nameStr);
        end

        switch nameStr
            case "timeoutSec"
                obj.pPar.timeoutSec = double(value);

            case "fpsAlpha"
                obj.pPar.fpsAlpha = double(value);

            case "fps"
                obj.pPar.fps = double(value);

            case "nodeName"
                obj.pPar.nodeName = string(value);

            case "topicImage"
                obj.pPar.topicImage = string(value);

            case "topicImageRight"
                obj.pPar.topicImageRight = string(value);

            case "topicDepth"
                obj.pPar.topicDepth = string(value);

            case "topicCameraInfo"
                obj.pPar.topicCameraInfo = string(value);

            case "enableImu"
                obj.pPar.enableImu = logical(value);

            case "topicImu"
                obj.pPar.topicImu = string(value);

            case "enablePose"
                obj.pPar.enablePose = logical(value);

            case "topicPose"
                obj.pPar.topicPose = string(value);

            case "topicOdom"
                obj.pPar.topicOdom = string(value);

            case "enablePointCloud"
                obj.pPar.enablePointCloud = logical(value);

            case "topicPointCloud"
                obj.pPar.topicPointCloud = string(value);
        end
    end
end

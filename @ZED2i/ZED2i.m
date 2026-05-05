classdef ZED2i < handle
    %ZED2i ZED2i MATLAB wrapper (ROS2) with scheduled acquisition.

    properties
        pPar 
        pCom 
        pData
        pFlag
        pPos 
        pCAD 
    end

    methods
        function obj = ZED2i(camId, varargin)
        %ZED2i ZED2i ROS2 MATLAB wrapper (stateful).
        %
        % Usage:
        %   zed = ZED2i(0);
        %   zed = ZED2i(0, "Profile", "calibration");
        %
        % The constructor MUST initialize:
        %   - pPar (parameters)
        %   - pCom, pData, pFlag, pPos (state)
        %
        % All configuration should be driven by the class, not demos.

            if nargin < 1 || isempty(camId)
                camId = 0;
            end

            p = inputParser;
            addParameter(p, "Profile", "live", @(x) isstring(x) || ischar(x));
            parse(p, varargin{:});
            profile = string(p.Results.Profile);

            % ---------- Always create parameters first ----------
            obj.iParameters(camId);

            % ---------- Then create state containers ----------
            obj.iState();
            obj.iControlVariables();

            % ---------- Apply profile AFTER params+state exist ----------
            obj.configure(profile);
        end

        function configure(obj, profile)
            %configure Apply a predefined configuration profile.
            %
            % This keeps demos generic: they just call configure("calibration") etc.

            profile = string(profile);

            switch lower(profile)
                case {"live", "default"}
                    obj.setStreamHz("image", 30);
                    obj.setStreamHz("depth", 30);
                    obj.setStreamHz("imu",   120);
                    obj.setStreamHz("pose",  10);
                    obj.setStreamHz("pcd",   1);
                    obj.setStreamHz("calib", 0);

                    obj.setEnabled("enableImu", true);
                    obj.setEnabled("enablePose", true);
                    obj.setEnabled("enablePointCloud", false);

                case "calibration"
                    obj.setStreamHz("image", 0);
                    obj.setStreamHz("depth", 0);
                    obj.setStreamHz("imu",   0);
                    obj.setStreamHz("pose",  0);
                    obj.setStreamHz("pcd",   0);
                    obj.setStreamHz("calib", Inf);

                    obj.setEnabled("enableImu", false);
                    obj.setEnabled("enablePose", false);
                    obj.setEnabled("enablePointCloud", false);

                case "minimal"
                    obj.setStreamHz("image", 15);
                    obj.setStreamHz("depth", 15);
                    obj.setStreamHz("imu",   0);
                    obj.setStreamHz("pose",  0);
                    obj.setStreamHz("pcd",   0);
                    obj.setStreamHz("calib", 0);

                    obj.setEnabled("enableImu", false);
                    obj.setEnabled("enablePose", false);
                    obj.setEnabled("enablePointCloud", false);

                otherwise
                    error("ZED2i:configure:InvalidProfile", ...
                        "Unknown profile '%s'. Use: live, calibration, or minimal.", ...
                        char(profile));
            end
        end

        function setStreamHz(obj, streamName, hz)
            streamName = string(streamName);

            validateattributes(streamName, {'string','char'}, {'scalartext'});
            validateattributes(hz, {'double','single'}, {'scalar','real','nonnegative'});

            if ~isfield(obj.pPar, "streamHz") || ~isstruct(obj.pPar.streamHz)
                obj.pPar.streamHz = struct();
            end

            obj.pPar.streamHz.(char(streamName)) = double(hz);
        end



        function setParameters(obj, overrides)
            %setParameters Safe parameter merge without exposing internals to demos.

            if ~isstruct(overrides) || ~isscalar(overrides)
                error("ZED2i:setParameters:InvalidInput", ...
                    "Overrides must be a scalar struct.");
            end

            f = fieldnames(overrides);
            for k = 1:numel(f)
                key = f{k};
                obj.pPar.(key) = overrides.(key); %#ok<AGROW>
            end

            % Ensure required fields exist even after overrides
            obj.ensureCoreDefaults_();
        end

        function setEnabled(obj, fieldName, value)
            %setEnabled Configure enable flags in pPar (boolean).

            fieldName = string(fieldName);
            value = logical(value);

            obj.pPar.(char(fieldName)) = value;
        end

        function tf = isStreamEnabled(obj, streamName)
            %isStreamEnabled True if streamHz is Inf or > 0.
            streamName = char(string(streamName));
            obj.ensureStreamHzStruct_();
            if ~isfield(obj.pPar.streamHz, streamName)
                tf = false;
                return;
            end
            hz = double(obj.pPar.streamHz.(streamName));
            tf = isinf(hz) || (hz > 0);
        end
    end

    methods (Access = private)
        function ensureStreamHzStruct_(obj)
            if isempty(obj.pPar) || ~isstruct(obj.pPar)
                obj.pPar = struct();
            end
            if ~isfield(obj.pPar, "streamHz") || ~isstruct(obj.pPar.streamHz)
                obj.pPar.streamHz = struct();
            end

            % Ensure known keys exist (stable contract)
            defaults = struct( ...
                "image", 15, ...
                "depth", 15, ...
                "imu",   100, ...
                "pose",  15, ...
                "pcd",   1, ...
                "calib", 0 ...
            );

            dkeys = fieldnames(defaults);
            for i = 1:numel(dkeys)
                k = dkeys{i};
                if ~isfield(obj.pPar.streamHz, k)
                    obj.pPar.streamHz.(k) = defaults.(k);
                end
            end
        end

        function ensureCoreDefaults_(obj)
            % Make sure essential params exist (defensive).

            if ~isfield(obj.pPar, "timeoutSec") || isempty(obj.pPar.timeoutSec)
                obj.pPar.timeoutSec = 1.0;
            end
            if ~isfield(obj.pPar, "fpsAlpha") || isempty(obj.pPar.fpsAlpha)
                obj.pPar.fpsAlpha = 0.2;
            end

            % enable flags defaults
            if ~isfield(obj.pPar, "enableImu"),        obj.pPar.enableImu = true; end
            if ~isfield(obj.pPar, "enablePose"),       obj.pPar.enablePose = true; end
            if ~isfield(obj.pPar, "enablePointCloud"), obj.pPar.enablePointCloud = true; end

            obj.ensureStreamHzStruct_();
        end
    end
end

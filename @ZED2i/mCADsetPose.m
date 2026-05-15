function mCADsetPose(zed, position, eulerRPY, varargin)
%mCADsetPose Set the CAD pose and update the ZED2i model.
%
% Syntax:
%   zed.mCADsetPose(position, eulerRPY)
%   zed.mCADsetPose(position, eulerRPY, "Offset", [0 0 0])
%   zed.mCADsetPose(position, eulerRPY, "ShowLines", false)
%
% Inputs:
%   position : 1x3 or 3x1 position [x y z] in meters.
%   eulerRPY : 1x3 or 3x1 Euler angles [roll pitch yaw] in radians.
%
% Name-Value:
%   Offset    : 1x3 translation offset applied to position.
%   ShowLines : logical flag for debug line visualization.
%
% Notes:
%   - The CAD pose convention follows:
%       zed.pPos.X = [x y z roll pitch yaw ...]^T
%   - Scaling should preferably be configured before the first CAD load
%     through zed.pPar.cad.scale.

    parser = inputParser;
    addParameter(parser, "Offset", [0 0 0], ...
        @(x) isnumeric(x) && numel(x) == 3);
    addParameter(parser, "ShowLines", false, ...
        @(x) islogical(x) || isnumeric(x));
    parse(parser, varargin{:});

    offset = double(parser.Results.Offset(:));
    showLines = logical(parser.Results.ShowLines);

    if nargin < 3
        error("ZED2i:CAD:InvalidInput", ...
            "position and eulerRPY must be provided.");
    end

    position = double(position(:));
    eulerRPY = double(eulerRPY(:));

    if numel(position) ~= 3
        error("ZED2i:CAD:InvalidPosition", ...
            "position must have 3 elements: [x y z].");
    end

    if numel(eulerRPY) ~= 3
        error("ZED2i:CAD:InvalidOrientation", ...
            "eulerRPY must have 3 elements: [roll pitch yaw].");
    end

    if any(~isfinite(position)) || any(~isfinite(eulerRPY))
        error("ZED2i:CAD:InvalidPose", ...
            "position and eulerRPY must contain finite values.");
    end

    if isempty(zed.pPos) || ~isstruct(zed.pPos)
        zed.pPos = struct();
    end

    if ~isfield(zed.pPos, "X") || isempty(zed.pPos.X) || numel(zed.pPos.X) < 12
        zed.pPos.X = zeros(12, 1);
    else
        zed.pPos.X = double(zed.pPos.X(:));
        if numel(zed.pPos.X) < 12
            zed.pPos.X(end + 1:12, 1) = 0;
        end
    end

    position = position + offset;

    zed.pPos.X(1:3) = position;
    zed.pPos.X(4:6) = eulerRPY;

    zed.mCADplot(showLines);
end
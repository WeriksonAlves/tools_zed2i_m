function mCADplot(zed, showLines)
% mCADplot Plot/update ZED2i CAD model at current pose.
%
% Pose convention:
%   zed.pPos.X = [x y z roll pitch yaw ...]^T
%
% Optimizations:
% - Lazy CAD load.
% - Create graphics once (patch handles).
% - Per-frame update uses affine transform: Vnew = V*R' + t'
%   avoiding homogeneous coordinates and large temporary allocations.
% - Early-exit if pose did not change.

    if nargin < 2
        showLines = false;
    end

    ensureCadLoaded_(zed);
    ensurePose_(zed);

    zed.pCAD.flagLines = logical(showLines);

    if zed.pCAD.flagLines
        updateDebugLine_(zed);
    end

    if ~isfield(zed.pCAD, "flagCreated") || zed.pCAD.flagCreated == 0
        mCADmake_(zed);
        finalizeVisibility_(zed);
        drawnow limitrate nocallbacks
        return;
    end

    updated = updateCadPose_(zed);
    if updated
        drawnow limitrate nocallbacks
    end
end

% =========================================================================
% Helpers
% =========================================================================
function ensureCadLoaded_(zed)
    if ~isfield(zed, "pCAD") || isempty(zed.pCAD)
        zed.pCAD = struct();
    end

    if ~isfield(zed.pCAD, "flagLoaded") || zed.pCAD.flagLoaded ~= 1
        mCADload(zed);
    end

    if ~isfield(zed.pCAD, "flagLoaded") || zed.pCAD.flagLoaded ~= 1
        error("ZED2i:CAD:NotLoaded", ...
            "CAD model could not be loaded. Check zed.pPar.cad settings.");
    end
end

function ensurePose_(zed)
    if ~isfield(zed, "pPos") || ~isfield(zed.pPos, "X") || isempty(zed.pPos.X)
        zed.pPos.X = zeros(12, 1);
    end

    X = zed.pPos.X(:);
    if ~isnumeric(X)
        error("ZED2i:CAD:InvalidPoseType", "zed.pPos.X must be numeric.");
    end
    if numel(X) < 6
        error("ZED2i:CAD:MissingPose", ...
            "zed.pPos.X has %d elements (need >= 6).", numel(X));
    end

    % Normalize to double column vector for pose (cheap, tiny)
    zed.pPos.X = double(X);
end

function updateDebugLine_(zed)
    x = zed.pPos.X(1);
    y = zed.pPos.X(2);
    z = zed.pPos.X(3);

    if ~isfield(zed.pCAD, "i2D") || ~isfield(zed.pCAD.i2D, "Vertices")
        zed.pCAD.i2D = struct();
    end

    zed.pCAD.i2D.Vertices = [x x x + 1e-6; y y y; 0 z z]';
end

function finalizeVisibility_(zed)
    for idx = 1:numel(zed.pCAD.i3D)
        zed.pCAD.i3D{idx}.FaceAlpha = 1.0;
        zed.pCAD.i3D{idx}.Visible = "on";
    end
end

function updated = updateCadPose_(zed)
% updateCadPose_ Updates patch vertices using cached base vertices.
%
% Returns:
%   updated (logical): true if an update was applied.

    updated = false;

    x = zed.pPos.X(1);
    y = zed.pPos.X(2);
    z = zed.pPos.X(3);

    roll  = zed.pPos.X(4);
    pitch = zed.pPos.X(5);
    yaw   = zed.pPos.X(6);

    pose6 = [x; y; z; roll; pitch; yaw];

    % Early exit if pose unchanged (within tolerance)
    tol = 1e-12;
    if isfield(zed.pCAD, "lastPose6") && numel(zed.pCAD.lastPose6) == 6
        if all(abs(zed.pCAD.lastPose6 - pose6) < tol)
            return;
        end
    end
    zed.pCAD.lastPose6 = pose6;
    updated = true;

    % Rotation matrix (ZYX)
    cr = cos(roll);  sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw);   sy = sin(yaw);

    % R = Rz * Ry * Rx
    R = [ ...
        cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr; ...
        sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr; ...
        -sp,   cp*sr,            cp*cr ...
    ];

    t = [x; y; z];

    % Match numeric type of cached vertices
    baseV = zed.pCAD.baseVertices{1};
    if isa(baseV, "single")
        R = single(R);
        t = single(t);
    end

    Rt = R.';     % for (N×3)*(3×3)
    tt = t.';     % 1×3 for broadcasting

    for i = 1:numel(zed.pCAD.i3D)
        V0 = zed.pCAD.baseVertices{i};     % N×3
        Vn = V0 * Rt + tt;                 % N×3
        set(zed.pCAD.i3D{i}, "Vertices", Vn);
    end
end

% =========================================================================
% Local helper: create patch objects once
% =========================================================================
function mCADmake_(zed)
    nParts = numel(zed.pCAD.obj);

    zed.pCAD.i3D = cell(1, nParts);
    zed.pCAD.baseVertices = cell(1, nParts);

    ax = gca;

    for i = 1:nParts
        obj = zed.pCAD.obj{i};
        mtl = zed.pCAD.mtl{i};

        % Cache base vertices as N×3 (patch expects that format)
        V = obj.v;
        if size(V, 1) == 3
            Vn3 = V.';  % N×3
        else
            Vn3 = V;    % already N×3
        end

        % Preserve numeric type (single helps memory)
        zed.pCAD.baseVertices{i} = Vn3;

        faces = obj.f3;
        if size(faces, 1) == 3
            faces = faces.'; % M×3
        end

        h = patch( ...
            "Parent", ax, ...
            "Vertices", Vn3, ...
            "Faces", faces ...
        );

        fvcd3 = buildFaceColorsFast_(obj, mtl);
        h.FaceVertexCData = fvcd3;
        h.FaceColor = "flat";
        h.EdgeColor = "none";
        h.FaceAlpha = 0.0;
        h.Visible = "off";

        zed.pCAD.i3D{i} = h;
    end

    zed.pCAD.flagCreated = 1;
end

function fvcd3 = buildFaceColorsFast_(obj, mtl)
% buildFaceColorsFast_ Build per-face colors with O(F+M).
%
% Output:
%   fvcd3: M×3

    nFaces = numel(obj.umat3);
    fvcd3 = zeros(nFaces, 3);

    if isempty(mtl)
        fvcd3(:, :) = repmat([0.7, 0.7, 0.7], nFaces, 1);
        return;
    end

    % Map material name -> diffuse color (Kd)
    nameToKd = containers.Map("KeyType", "char", "ValueType", "any");
    for j = 1:numel(mtl)
        nameToKd(char(mtl(j).name)) = double(mtl(j).Kd(:)).';
    end

    usemtl = obj.usemtl;
    if isempty(usemtl)
        usemtl = "default";
    end

    defaultKd = [0.7, 0.7, 0.7];

    for i = 1:nFaces
        useIdx = obj.umat3(i);

        if useIdx < 1 || useIdx > numel(usemtl)
            fvcd3(i, :) = defaultKd;
            continue;
        end

        nameWanted = char(usemtl(useIdx));
        if nameToKd.isKey(nameWanted)
            fvcd3(i, :) = nameToKd(nameWanted);
        else
            fvcd3(i, :) = defaultKd;
        end
    end
end

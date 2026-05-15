function mCADload(zed)
% mCADload Load 3D CAD model(s) for the ZED2i class.
%
% Optimizations:
% - Applies defaults once.
% - Caches loaded + transformed meshes (persistent cache).
% - Reduces memory by optionally storing vertices as single.
% - Avoids allocating ones(1,N) for translations (uses broadcasting).

    zed.pPar.cad = applyCadDefaults_(zed);

    cad = zed.pPar.cad;

    if ~cad.enable
        zed.pCAD.flagLoaded  = 0;
        zed.pCAD.flagCreated = 0;
        zed.pCAD.flagLines   = 0;
        return;
    end

    parts = cad.parts;
    if ~isstruct(parts)
        error("ZED2i:CAD:InvalidParts", ...
            "zed.pPar.cad.parts must be a struct or struct array.");
    end

    % Persistent cache: key -> struct with obj/mtl/parts after transform
    persistent cadCache
    if isempty(cadCache)
        cadCache = containers.Map("KeyType", "char", "ValueType", "any");
    end

    zed.pCAD.obj = cell(1, numel(parts));
    zed.pCAD.mtl = cell(1, numel(parts));
    zed.pCAD.parts = repmat(struct("t_body", [0; 0; 0]), 1, numel(parts));

    for i = 1:numel(parts)
        part = parts(i);

        validateCadPart_(part);

        objPath = fullfile(cad.modelDir, part.obj);
        mtlPath = fullfile(cad.modelDir, part.mtl);

        if exist(objPath, "file") ~= 2
            error("ZED2i:CAD:ObjNotFound", "OBJ file not found: %s", objPath);
        end
        if exist(mtlPath, "file") ~= 2
            error("ZED2i:CAD:MtlNotFound", "MTL file not found: %s", mtlPath);
        end

        tBody = [0; 0; 0];
        if isfield(part, "t_body") && ~isempty(part.t_body)
            tBody = double(part.t_body(:));
        end

        cacheKey = makeCadCacheKey_(objPath, mtlPath, cad, tBody);
        if cadCache.isKey(cacheKey)
            cached = cadCache(cacheKey);
            zed.pCAD.obj{i} = cached.obj;
            zed.pCAD.mtl{i} = cached.mtl;
            zed.pCAD.parts(i).t_body = cached.t_body;
            continue;
        end

        obj = localLoadObjFast_(objPath);
        mtl = localLoadMtlFast_(mtlPath);

        V = double(obj.v);

        if cad.center
            vMin = min(V, [], 2);
            vMax = max(V, [], 2);
            V = V - 0.5 * (vMin + vMax);
        end

        V = V .* cad.scale;
        V = cad.R_model_to_body * V;

        % Translation: broadcast 3x1 over 3xN
        V = V + tBody;

        if cad.useSingle
            obj.v = single(V);
        else
            obj.v = V;
        end

        cached = struct();
        cached.obj = obj;
        cached.mtl = mtl;
        cached.t_body = tBody;

        cadCache(cacheKey) = cached;

        zed.pCAD.obj{i} = obj;
        zed.pCAD.mtl{i} = mtl;
        zed.pCAD.parts(i).t_body = tBody;
    end

    zed.pCAD.flagLoaded  = 1;
    zed.pCAD.flagCreated = 0;
    zed.pCAD.flagLines   = 0;
end

% -------------------------------------------------------------------------
% Local helpers
% -------------------------------------------------------------------------
function cad = applyCadDefaults_(zed)
    if isempty(zed.pPar) || ~isstruct(zed.pPar)
        zed.pPar = struct();
    end

    if ~isfield(zed.pPar, "cad") || isempty(zed.pPar.cad) || ~isstruct(zed.pPar.cad)
        zed.pPar.cad = struct();
    end

    cad = zed.pPar.cad;

    cad = setDefault_(cad, "enable", true);
    cad = setDefault_(cad, "scale", 0.01);
    cad = setDefault_(cad, "center", true);
    cad = setDefault_(cad, "R_model_to_body", defaultRModelToBody_());
    cad = setDefault_(cad, "useSingle", true);

    if ~isfield(cad, "modelDir") || strlength(string(cad.modelDir)) == 0
        classDir = fileparts(mfilename("fullpath"));   % .../@ZED2i
        repoRoot = fileparts(classDir);                % .../tools_zed2i_m
        cand = fullfile(repoRoot, "CADdata");

        if exist(cand, "dir") == 7
            cad.modelDir = cand;
        else
            cad.modelDir = classDir;
        end
    end

    if ~isfield(cad, "parts") || isempty(cad.parts)
        cad.parts = struct( ...
            "obj", "ZED2i.obj", ...
            "mtl", "ZED2i.mtl", ...
            "t_body", [0; 0; 0] ...
        );
    end

    zed.pPar.cad = cad;
end

function cad = setDefault_(cad, fieldName, defaultValue)
    if ~isfield(cad, fieldName) || isempty(cad.(fieldName))
        cad.(fieldName) = defaultValue;
    end
end

function validateCadPart_(part)
    if ~isfield(part, "obj") || ~isfield(part, "mtl")
        error("ZED2i:CAD:InvalidPart", ...
            "Each CAD part must define .obj and .mtl.");
    end
end

function key = makeCadCacheKey_(objPath, mtlPath, cad, tBody)
    % Key includes transformation parameters and storage mode.
    key = sprintf( ...
        "%s|%s|scale=%.8g|center=%d|useSingle=%d|R=%s|t=%s", ...
        objPath, mtlPath, cad.scale, cad.center, cad.useSingle, ...
        mat2str(cad.R_model_to_body, 8), mat2str(tBody.', 8));
end

% -------------------------------------------------------------------------
% Fast OBJ/MTL loaders (minimal fields expected by mCADplot)
% -------------------------------------------------------------------------
function obj = localLoadObjFast_(objPath)
% Minimal OBJ loader:
% - vertices (v)
% - triangular faces (f3)
% - per-face material index (umat3) with a name list (usemtl)
%
% Faster than split-based parsing for typical meshes.

    fid = fopen(objPath, "r");
    if fid < 0
        error("ZED2i:CAD:ObjOpenFail", "Failed to open OBJ: %s", objPath);
    end

    % Chunked growth to avoid O(n^2) reallocations from end+1
    vChunk = 4096;
    fChunk = 4096;

    V = zeros(3, vChunk);
    F3 = zeros(3, fChunk, "uint32");
    umat3 = zeros(fChunk, 1, "uint32");

    vCount = 0;
    fCount = 0;

    usemtlNames = strings(0, 1);
    mtlMap = containers.Map("KeyType", "char", "ValueType", "uint32");
    currentMtlIdx = uint32(1);

    try
        while true
            ln = fgetl(fid);
            if ~ischar(ln)
                break;
            end

            ln = strtrim(ln);
            if ln == "" || startsWith(ln, "#")
                continue;
            end

            if startsWith(ln, "v ")
                nums = sscanf(ln(2:end), "%f");
                if numel(nums) >= 3
                    vCount = vCount + 1;
                    if vCount > size(V, 2)
                        V(:, end+1:end+vChunk) = 0; %#ok<AGROW>
                    end
                    V(:, vCount) = nums(1:3);
                end

            elseif startsWith(ln, "f ")
                idx = parseObjFaceVertexIndices_(ln(2:end), vCount);

                if numel(idx) == 3
                    [F3, umat3, fCount] = pushTri_( ...
                        F3, umat3, fCount, idx, currentMtlIdx, fChunk);

                elseif numel(idx) > 3
                    % Fan triangulation for polygons with more than 3 vertices.
                    for k = 2:(numel(idx) - 1)
                        tri = [idx(1); idx(k); idx(k + 1)];
                        [F3, umat3, fCount] = pushTri_( ...
                            F3, umat3, fCount, tri, currentMtlIdx, fChunk);
                    end
                end
            end
        end
    catch ME
        fclose(fid);
        rethrow(ME);
    end

    fclose(fid);

    V = V(:, 1:vCount);
    F3 = F3(:, 1:fCount);
    umat3 = umat3(1:fCount);

    if isempty(usemtlNames)
        usemtlNames = "default";
    end

    obj = struct();
    obj.v = V;
    obj.f3 = double(F3);         % keep compatibility (many plotting codes expect double)
    obj.umat3 = double(umat3);
    obj.usemtl = usemtlNames;
end

function [F3, umat3, fCount] = pushTri_(F3, umat3, fCount, tri, mtlIdx, fChunk)
    fCount = fCount + 1;
    if fCount > size(F3, 2)
        F3(:, end+1:end+fChunk) = 0; %#ok<AGROW>
        umat3(end+1:end+fChunk, 1) = 0; %#ok<AGROW>
    end
    F3(:, fCount) = tri(:);
    umat3(fCount, 1) = mtlIdx;
end

function mtl = localLoadMtlFast_(mtlPath)
% Minimal MTL loader: reads "newmtl" blocks and Kd diffuse color.

    fid = fopen(mtlPath, "r");
    if fid < 0
        error("ZED2i:CAD:MtlOpenFail", "Failed to open MTL: %s", mtlPath);
    end

    mtl = struct("name", {}, "Kd", {});
    current = struct("name", "", "Kd", [0.7; 0.7; 0.7]);

    try
        while true
            ln = fgetl(fid);
            if ~ischar(ln)
                break;
            end

            ln = strtrim(ln);
            if ln == "" || startsWith(ln, "#")
                continue;
            end

            if startsWith(ln, "newmtl ")
                if strlength(string(current.name)) > 0
                    mtl(end+1) = current; %#ok<AGROW>
                end
                current = struct("name", string(strtrim(ln(7:end))), "Kd", [0.7; 0.7; 0.7]);

            elseif startsWith(ln, "Kd ")
                nums = sscanf(ln(2:end), "%f");
                if numel(nums) >= 3
                    current.Kd = nums(1:3);
                end
            end
        end
    catch ME
        fclose(fid);
        rethrow(ME);
    end

    fclose(fid);

    if strlength(string(current.name)) > 0
        mtl(end+1) = current; %#ok<AGROW>
    end

    if isempty(mtl)
        mtl = struct("name", "default", "Kd", [0.7; 0.7; 0.7]);
    end
end

function idx = parseObjFaceVertexIndices_(faceLine, vertexCount)
%parseObjFaceVertexIndices_ Extract only vertex indices from an OBJ face line.
%
% Supports:
%   f v1 v2 v3
%   f v1/vt1 v2/vt2 v3/vt3
%   f v1//vn1 v2//vn2 v3//vn3
%   f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3
%
% Also supports negative OBJ indices.

    tokens = regexp(strtrim(faceLine), "\s+", "split");
    idx = zeros(numel(tokens), 1, "uint32");

    validCount = 0;

    for i = 1:numel(tokens)
        token = tokens{i};

        if isempty(token)
            continue;
        end

        parts = regexp(token, "/", "split");
        vertexIndex = str2double(parts{1});

        if isnan(vertexIndex) || vertexIndex == 0
            continue;
        end

        % OBJ negative indices are relative to the current vertex count.
        if vertexIndex < 0
            vertexIndex = double(vertexCount) + vertexIndex + 1;
        end

        if vertexIndex < 1 || vertexIndex > vertexCount
            warning("ZED2i:CAD:InvalidFaceIndex", ...
                "Ignoring invalid face index %d. Valid range is [1, %d].", ...
                vertexIndex, vertexCount);
            idx = zeros(0, 1, "uint32");
            return;
        end

        validCount = validCount + 1;
        idx(validCount) = uint32(vertexIndex);
    end

    idx = idx(1:validCount);
end

function R = defaultRModelToBody_()
%defaultRModelToBody_ Default fixed rotation from OBJ frame to ZED body frame.
%
% The ZED2i OBJ model frame is not necessarily aligned with the odometry/body
% frame used in the live visualization. This rotation corrects the default
% visual orientation of the CAD model.

    theta = -pi / 2;

    R = [ ...
        cos(theta), -sin(theta), 0; ...
        sin(theta),  cos(theta), 0; ...
        0,           0,          1 ...
    ];
end

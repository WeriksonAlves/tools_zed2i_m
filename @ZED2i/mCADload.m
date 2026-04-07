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
    cad = setDefault_(cad, "scale", 1.0);
    cad = setDefault_(cad, "center", true);
    cad = setDefault_(cad, "R_model_to_body", eye(3));
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

            elseif startsWith(ln, "usemtl ")
                mtlName = char(strtrim(ln(7:end)));
                if ~mtlMap.isKey(mtlName)
                    usemtlNames(end+1, 1) = string(mtlName); %#ok<AGROW>
                    mtlMap(mtlName) = uint32(numel(usemtlNames));
                end
                currentMtlIdx = mtlMap(mtlName);

            elseif startsWith(ln, "f ")
                % Extract first index of each vertex token using regexp:
                % captures numbers before any '/'
                tok = regexp(ln(2:end), "(\d+)(?=(/|\s|$))", "tokens");
                if isempty(tok)
                    continue;
                end
                idx = zeros(numel(tok), 1, "uint32");
                for k = 1:numel(tok)
                    idx(k) = uint32(str2double(tok{k}{1}));
                end

                if numel(idx) == 3
                    [F3, umat3, fCount] = pushTri_(F3, umat3, fCount, idx, currentMtlIdx, fChunk);
                elseif numel(idx) > 3
                    for k = 2:(numel(idx) - 1)
                        tri = [idx(1); idx(k); idx(k + 1)];
                        [F3, umat3, fCount] = pushTri_(F3, umat3, fCount, tri, currentMtlIdx, fChunk);
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

function mCADdel(zed)
% mCADdel Delete CAD patches for the ZED2i model.

    if ~isfield(zed, "pCAD") || isempty(zed.pCAD)
        return;
    end

    if isfield(zed.pCAD, "i3D")
        try
            for i = 1:numel(zed.pCAD.i3D)
                if isgraphics(zed.pCAD.i3D{i})
                    delete(zed.pCAD.i3D{i});
                end
            end
        catch
            % Best-effort cleanup; ignore errors
        end
        zed.pCAD = rmfield(zed.pCAD, "i3D");
    end

    if isfield(zed.pCAD, "flagCreated")
        zed.pCAD.flagCreated = 0;
    end
end

function mCADcolor(zed, rgb)
% mCADcolor Set a uniform color for the CAD model.
%
% rgb: 1x3 or 3x1 (values in [0,1])

    if nargin < 2 || isempty(rgb)
        return;
    end

    if ~isfield(zed, "pCAD") || ~isfield(zed.pCAD, "i3D")
        error("ZED2i:CAD:NotCreated", "CAD patches not created. Call mCADplot(zed) at least once.");
    end

    c = double(rgb(:)');
    if numel(c) ~= 3
        error("ZED2i:CAD:InvalidColor", "rgb must have 3 elements.");
    end

    % Force uniform color (robust, independent of .mtl content)
    for i = 1:numel(zed.pCAD.i3D)
        if isgraphics(zed.pCAD.i3D{i})
            zed.pCAD.i3D{i}.FaceColor = c;
            zed.pCAD.i3D{i}.FaceVertexCData = [];
        end
    end
end

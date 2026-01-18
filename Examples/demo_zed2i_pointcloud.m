% demo_zed2i_pointcloud.m
% PointCloud2 snapshot and basic visualization.

clearvars;
close all;
clc;

zed = ZED2i("enablePointCloud", true);
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

fprintf("Waiting for point cloud...\n");
pc = zed.rGetPointCloud();

if ~zed.pFlag.HasPointCloud || isempty(pc.XYZ)
    fprintf("Point cloud not available. LastError: %s\n", string(zed.pFlag.LastError));
    return;
end

fprintf("Point cloud received: %d points\n", size(pc.XYZ, 1));

% Se tiver Computer Vision Toolbox:
try
    ptCloud = pointCloud(pc.XYZ);
    figure('Name', 'ZED2i PointCloud', 'NumberTitle', 'off');
    pcshow(ptCloud);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('ZED2i Registered PointCloud');
catch excp
    fprintf("Could not create pointCloud object or visualize. Reason: %s\n", excp.message);
end

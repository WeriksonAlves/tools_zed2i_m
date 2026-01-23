% demo_zed2i_pointcloud.m
% PointCloud2 snapshot and basic visualization using ZED2i.rGetPointCloud.
%
% - Habilita point cloud via construtor (enablePointCloud=true)
% - Aguarda alguns segundos por uma nuvem válida
% - Faz visualização básica usando pointCloud/pcshow (se disponível)

clearvars;
close all;
clc;

%% Add project to path (root-based)
PastaAtual = pwd;
PastaRaiz  = 'elt793'; % tools_zed2i_m

idx = strfind(PastaAtual, PastaRaiz);
if ~isempty(idx)
    rootPath = PastaAtual(1:(idx(1) + numel(PastaRaiz) - 1));
    cd(rootPath);
    addpath(genpath(pwd));
    cd(PastaAtual);
else
    % Se não encontrar a pasta raiz, ainda assim adiciona o path atual
    addpath(genpath(PastaAtual));
end

%% Create ZED2i object with point cloud enabled
zed = ZED2i("enablePointCloud", true);
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

%% Wait for a valid point cloud snapshot
fprintf("Waiting for point cloud...\n");

pc     = struct();
t_max  = 10;   % total waiting time [s]
rate   = 2;    % attempts per second
t0     = tic;
tc     = tic;

while toc(t0) < t_max
    % Simple rate control
    if toc(tc) < 1 / rate
        pause(0.01);
        continue;
    end
    tc = tic;

    pc = zed.rGetPointCloud();

    if zed.pFlag.HasPointCloud && isfield(pc, "XYZ") && ~isempty(pc.XYZ)
        break;
    end
end

% Check if we got something
if ~zed.pFlag.HasPointCloud || ~isfield(pc, "XYZ") || isempty(pc.XYZ)
    fprintf("Point cloud not available. LastError: %s\n", string(zed.pFlag.LastError));
    return;
end

numPoints = size(pc.XYZ, 1);
fprintf("Point cloud received: %d points\n", numPoints);

%% Optional: subsample for visualization (avoid overloading pcshow)
maxPointsToShow = 1e5;

xyzVis = pc.XYZ;
if numPoints > maxPointsToShow
    idx = randperm(numPoints, maxPointsToShow);
    xyzVis = xyzVis(idx, :);
    fprintf("Subsampling to %d points for visualization.\n", maxPointsToShow);
end

%% Visualize (if Computer Vision Toolbox is available)
try
    ptCloud = pointCloud(xyzVis);
    figure('Name', 'ZED2i PointCloud', 'NumberTitle', 'off');
    pcshow(ptCloud);
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('ZED2i Registered PointCloud');
    axis equal;
catch excp
    fprintf("Could not create pointCloud object or visualize. Reason: %s\n", excp.message);
end

% rDisconnect será chamado automaticamente por cleanupObj

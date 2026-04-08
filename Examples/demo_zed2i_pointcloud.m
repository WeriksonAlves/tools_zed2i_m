% demo_zed2i_pointcloud.m
% PointCloud2 real-time preview (10 s) using ZED2i.
%
% Dois modos de aquisição:
%   1) Explícito: zed.rosGetPointCloud()
%   2) Alto nível: zed.getSensorData() + autoFetchPointCloud = true
%
% Durante t_max segundos, o script:
%   - lê a point cloud
%   - subamostra para visualização
%   - atualiza um scatter3 em tempo real

%% Add project to path (root-based)
clearvars; close all; clc;
current_path = pwd;
root  = 'tools_zed2i_m';

idx = strfind(current_path, root);
if ~isempty(idx)
    rootPath = current_path(1:(idx(1) + numel(root) - 1));
    cd(rootPath);
    addpath(genpath(pwd));
    cd(current_path);
else
    addpath(genpath(current_path));
end

%% Create ZED2i object and guarantee proper cleanup
zed = ZED2i(0, "Profile", "live");

% Only demonstrate, since it is enabled by default in the "live" profile.
% But it's good to show how to enable or disable features.
zed.setEnabled("enableImu", false);
zed.setEnabled("enablePose", false);
zed.setEnabled("enablePointCloud", true);

zed.setStreamHz("pcd", 1);

cleanupObj = onCleanup(@() zed.rosDisconnect());
zed.rosConnect();

%% Buffers para logs de estado

%% Demo parameters
t_max = 10;
target_fps_cloud = 5;
maxPointsToShow = 1e4;

%% Initialization: get at least one valid point cloud
fprintf("Waiting for first valid point cloud...\n");

pc = zed.sEmptyPointCloud();
t0 = tic;
tc = tic;

while toc(t0) < t_max
    if toc(tc) < 1 / target_fps_cloud
        pause(0.01);
        continue;
    end
    tc = tic;

    data = zed.getSensorData();

    if ~data.Connected
        fprintf("Disconnected. LastError: %s\n", string(data.LastError));
        break;
    end

    if data.HasPointCloud && isfield(data.PointCloud, "XYZ") && ~isempty(data.PointCloud.XYZ)
        pc = data.PointCloud;
        fprintf("First PointCloud received via getSensorData().\n");
        break;
    end

    fprintf("Waiting for PointCloud...\n");
end

if ~isfield(pc, "XYZ") || isempty(pc.XYZ)
    fprintf("Could not get initial point cloud. LastError: %s\n", string(data.LastError));
    return;
end

% %% Preparar visualização inicial (scatter3)
% numPoints = size(pc.XYZ, 1);
% fprintf("Point cloud inicial: %d pontos.\n", numPoints);

% xyzVis = pc.XYZ;
% if numPoints > maxPointsToShow
%     idxSub = randperm(numPoints, maxPointsToShow);
%     xyzVis = xyzVis(idxSub, :);
%     fprintf("Subamostrando para %d pontos para visualização.\n", maxPointsToShow);
% end

% hFig = figure('Name', 'ZED2i PointCloud - Real-time', 'NumberTitle', 'off');
% hScat = scatter3(xyzVis(:,1), xyzVis(:,2), xyzVis(:,3), 1, xyzVis(:,3), '.');
% grid on;
% xlabel('X (m)');
% ylabel('Y (m)');
% zlabel('Z (m)');
% title(sprintf('ZED2i Registered PointCloud (%s)', ...
%     ternary(useHighLevel, "getSensorData", "rosGetPointCloud")));
% axis equal;

% % ----- Escalas FIXAS (cubo 4x4x4) -----
% xlim([-1 3]);
% ylim([-2 2]);
% zlim([-2 2]);

% axis manual;   % <-- CRÍTICO: trava o auto-scale
% colormap turbo;
% colorbar;

% drawnow;

% %% Loop de atualização em tempo real (10 s a partir de agora)
% t_startVis = tic;
% tc         = tic;

% fprintf("Iniciando visualização em tempo real por %.1f s...\n", t_max);

% while ishandle(hFig) && toc(t_startVis) < t_max

%     if toc(tc) < 1 / target_fps_cloud
%         pause(0.01);
%         continue;
%     end
%     tc = tic;

%     % Leitura de nova nuvem
%     if useHighLevel
%         data = zed.getSensorData();

%         if ~data.Connected
%             fprintf("ZED2i desconectado durante visualização. LastError: %s\n", ...
%                 string(data.LastError));
%             break;
%         end

%         if ~(data.HasPointCloud && isfield(data.PointCloud, "XYZ") ...
%                 && ~isempty(data.PointCloud.XYZ))
%             fprintf("PointCloud ainda não disponível neste frame (modo alto nível).\n");
%             continue;
%         end

%         pcFrame = data.PointCloud;

%     else
%         pcFrameLocal = zed.rosGetPointCloud();

%         if ~(zed.pFlag.HasPointCloud && isfield(pcFrameLocal, "XYZ") ...
%                 && ~isempty(pcFrameLocal.XYZ))
%             fprintf("PointCloud ainda não disponível neste frame (modo explícito).\n");
%             continue;
%         end

%         pcFrame = pcFrameLocal;
%     end

%     xyz = pcFrame.XYZ;
%     nFramePoints = size(xyz, 1);

%     % Subamostragem para visual
%     if nFramePoints > maxPointsToShow
%         idxSub = randperm(nFramePoints, maxPointsToShow);
%         xyzVis = xyz(idxSub, :);
%     else
%         xyzVis = xyz;
%     end

%     % Atualização do scatter
%     if ishandle(hScat)
%         set(hScat, ...
%             'XData', xyzVis(:,1), ...
%             'YData', xyzVis(:,2), ...
%             'ZData', xyzVis(:,3), ...
%             'CData', xyzVis(:,3));
%     end

%     titleStr = sprintf('ZED2i PointCloud (%s) | t = %.1f s | pts = %d', ...
%         ternary(useHighLevel, "getSensorData", "rosGetPointCloud"), ...
%         toc(t_startVis), nFramePoints);
%     title(titleStr);

%     drawnow limitrate nocallbacks;
% end

% fprintf("Visualização em tempo real encerrada.\n");

% % rosDisconnect será chamado automaticamente por cleanupObj

% %% Pequeno helper tipo operador ternário
% function out = ternary(cond, valTrue, valFalse)
%     if cond
%         out = valTrue;
%     else
%         out = valFalse;
%     end
% end

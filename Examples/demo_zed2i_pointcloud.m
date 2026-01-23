% demo_zed2i_pointcloud.m
% PointCloud2 real-time preview (10 s) using ZED2i.
%
% Dois modos de aquisição:
%   1) Explícito: zed.rGetPointCloud()
%   2) Alto nível: zed.rGetSensorData() + autoFetchPointCloud = true
%
% Durante t_max segundos, o script:
%   - lê a point cloud
%   - subamostra para visualização
%   - atualiza um scatter3 em tempo real

clearvars;
close all;
clc;

%% Add project to path (root-based)
PastaAtual = pwd;
PastaRaiz  = 'elt793';  % ajuste se o nome da raiz mudar

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

%% Escolha do modo de aquisição
fprintf("Selecione o modo de aquisição de PointCloud:\n");
fprintf("  1 - Explícito (rGetPointCloud)\n");
fprintf("  2 - Alto nível (rGetSensorData + autoFetchPointCloud)\n");

modo = input("Escolha [1/2]: ");

if isempty(modo) || ~ismember(modo, [1 2])
    fprintf("Escolha inválida. Usando modo 1 (explícito) por padrão.\n");
    modo = 1;
end

useHighLevel = (modo == 2);

if useHighLevel
    fprintf("\n[MODO 2] Usando rGetSensorData() com autoFetchPointCloud = true.\n\n");
else
    fprintf("\n[MODO 1] Usando rGetPointCloud() explícito.\n\n");
end

%% Criação do objeto ZED2i
if useHighLevel
    % Modo 2: autoFetchPointCloud ativado
    zed = ZED2i( ...
        "enablePointCloud",    true, ...
        "autoFetchPointCloud", true  ...
    );
else
    % Modo 1: fetch explícito
    zed = ZED2i( ...
        "enablePointCloud",    true, ...
        "autoFetchPointCloud", false ...
    );
end

cleanupObj = onCleanup(@() zed.rDisconnect());
zed.rConnect();

%% Parâmetros da preview
t_max           = 10;   % duração total [s]
target_fps_cloud = 5;   % atualizações de nuvem por segundo
maxPointsToShow = 1e4;  % limite de pontos no scatter (visualização)

%% Inicialização: obter pelo menos uma nuvem válida
fprintf("Aguardando primeira point cloud válida...\n");

pc  = struct();
t0  = tic;
tc  = tic;

while toc(t0) < t_max
    if toc(tc) < 1 / target_fps_cloud
        pause(0.01);
        continue;
    end
    tc = tic;

    if useHighLevel
        data = zed.rGetSensorData();

        if ~data.Connected
            fprintf("ZED2i desconectado. LastError: %s\n", string(data.LastError));
            break;
        end

        if data.HasPointCloud && isfield(data.PointCloud, "XYZ") ...
                && ~isempty(data.PointCloud.XYZ)
            pc = data.PointCloud;
            fprintf("Primeira PointCloud obtida via rGetSensorData().\n");
            break;
        else
            fprintf("Aguardando PointCloud via rGetSensorData()...\n");
        end
    else
        pcLocal = zed.rGetPointCloud();

        if zed.pFlag.HasPointCloud && isfield(pcLocal, "XYZ") ...
                && ~isempty(pcLocal.XYZ)
            pc = pcLocal;
            fprintf("Primeira PointCloud obtida via rGetPointCloud().\n");
            break;
        else
            fprintf("Aguardando PointCloud via rGetPointCloud()...\n");
        end
    end
end

if ~isfield(pc, "XYZ") || isempty(pc.XYZ)
    fprintf("Não foi possível obter uma point cloud inicial. LastError: %s\n", ...
        string(zed.pFlag.LastError));
    return;
end

%% Preparar visualização inicial (scatter3)
numPoints = size(pc.XYZ, 1);
fprintf("Point cloud inicial: %d pontos.\n", numPoints);

xyzVis = pc.XYZ;
if numPoints > maxPointsToShow
    idxSub = randperm(numPoints, maxPointsToShow);
    xyzVis = xyzVis(idxSub, :);
    fprintf("Subamostrando para %d pontos para visualização.\n", maxPointsToShow);
end

hFig = figure('Name', 'ZED2i PointCloud - Real-time', 'NumberTitle', 'off');
hScat = scatter3(xyzVis(:,1), xyzVis(:,2), xyzVis(:,3), 1, xyzVis(:,3), '.');
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title(sprintf('ZED2i Registered PointCloud (%s)', ...
    ternary(useHighLevel, "rGetSensorData", "rGetPointCloud")));
axis equal;

% ----- Escalas FIXAS (cubo 4x4x4) -----
xlim([-1 3]);
ylim([-2 2]);
zlim([-2 2]);

axis manual;   % <-- CRÍTICO: trava o auto-scale
colormap turbo;
colorbar;

drawnow;

%% Loop de atualização em tempo real (10 s a partir de agora)
t_startVis = tic;
tc         = tic;

fprintf("Iniciando visualização em tempo real por %.1f s...\n", t_max);

while ishandle(hFig) && toc(t_startVis) < t_max

    if toc(tc) < 1 / target_fps_cloud
        pause(0.01);
        continue;
    end
    tc = tic;

    % Leitura de nova nuvem
    if useHighLevel
        data = zed.rGetSensorData();

        if ~data.Connected
            fprintf("ZED2i desconectado durante visualização. LastError: %s\n", ...
                string(data.LastError));
            break;
        end

        if ~(data.HasPointCloud && isfield(data.PointCloud, "XYZ") ...
                && ~isempty(data.PointCloud.XYZ))
            fprintf("PointCloud ainda não disponível neste frame (modo alto nível).\n");
            continue;
        end

        pcFrame = data.PointCloud;

    else
        pcFrameLocal = zed.rGetPointCloud();

        if ~(zed.pFlag.HasPointCloud && isfield(pcFrameLocal, "XYZ") ...
                && ~isempty(pcFrameLocal.XYZ))
            fprintf("PointCloud ainda não disponível neste frame (modo explícito).\n");
            continue;
        end

        pcFrame = pcFrameLocal;
    end

    xyz = pcFrame.XYZ;
    nFramePoints = size(xyz, 1);

    % Subamostragem para visual
    if nFramePoints > maxPointsToShow
        idxSub = randperm(nFramePoints, maxPointsToShow);
        xyzVis = xyz(idxSub, :);
    else
        xyzVis = xyz;
    end

    % Atualização do scatter
    if ishandle(hScat)
        set(hScat, ...
            'XData', xyzVis(:,1), ...
            'YData', xyzVis(:,2), ...
            'ZData', xyzVis(:,3), ...
            'CData', xyzVis(:,3));
    end

    titleStr = sprintf('ZED2i PointCloud (%s) | t = %.1f s | pts = %d', ...
        ternary(useHighLevel, "rGetSensorData", "rGetPointCloud"), ...
        toc(t_startVis), nFramePoints);
    title(titleStr);

    drawnow limitrate nocallbacks;
end

fprintf("Visualização em tempo real encerrada.\n");

% rDisconnect será chamado automaticamente por cleanupObj

%% Pequeno helper tipo operador ternário
function out = ternary(cond, valTrue, valFalse)
    if cond
        out = valTrue;
    else
        out = valFalse;
    end
end

% demo_zed2i_rgbd.m
% RGB + Depth preview using ZED2i MATLAB ROS2 wrapper.

clearvars;
close all;
clc;

%% Add project to path (root-based)
PastaAtual = pwd;
PastaRaiz  = 'tools_zed2i_m';

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

%% Create ZED2i object and guarantee proper cleanup
zed = ZED2i();
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

%% Visualization and timing parameters
t_max      = 30;   % [s] total demo time
target_fps = 10;   % desired preview FPS

%% Pre-create figure and graphics objects
hFig = figure('Name', 'ZED2i RGB-D Preview', 'NumberTitle', 'off');

ax1 = subplot(1, 2, 1);
hImg = imshow(zeros(360, 640, 3, 'uint8'), 'Parent', ax1);
title(ax1, 'RGB');

ax2 = subplot(1, 2, 2);
hDepth = imagesc(zeros(360, 640, 'single'), 'Parent', ax2);
axis(ax2, 'image');
axis(ax2, 'off');
colormap(ax2, turbo);
cb = colorbar(ax2);
cb.Label.String = 'Depth (m)';
title(ax2, 'Depth');

% Initial depth scaling in meters
lo = 0.3;
hi = 6.0;
set(ax2, 'CLim', [lo hi]);

%% Main loop
tc = tic;          % frame pacing timer
t  = tic;          % total demo timer
frameCount = 0;

while ishandle(hFig) && toc(t) < t_max
    % Simple FPS limiter
    if toc(tc) > 1 / target_fps
        tc = tic;

        cycleTic = tic;

        % ---------------------------------------------------------------------
        % High-level snapshot: image, depth, metrics, etc.
        % ---------------------------------------------------------------------
        data = zed.rGetSensorData();

        if mod(frameCount, 30) == 0
            fprintf('\nrGetSensorData (last frame): %.3f s\n', toc(cycleTic));
        end

        if ~data.Connected
            % Se por algum motivo desconectou no meio, aborta demo
            if ~isempty(data.LastError)
                fprintf("[ZED2i] Disconnected: %s\n", char(data.LastError));
            end
            break;
        end

        % -------------------- RGB --------------------
        if data.HasImage && ~isempty(data.Image)
            set(hImg, 'CData', data.Image);
            title(ax1, sprintf('RGB | FPS: %.1f | Drops: %d', ...
                data.Metrics.ImageFps, data.Metrics.ImageDrops));
        else
            % Opcional: exibir informação de ausência de imagem
            title(ax1, 'RGB (no data)');
        end

        if mod(frameCount, 30) == 0
            fprintf('RGB (last frame): %.3f s\n', toc(cycleTic));
        end

        % -------------------- Depth --------------------
        if data.HasDepth && ~isempty(data.Depth)
            set(hDepth, 'CData', data.Depth);

            % Atualiza contraste a cada 10 frames usando apenas valores válidos
            if mod(frameCount, 10) == 0
                if isfield(data, "DepthMask") && ~isempty(data.DepthMask)
                    v = data.Depth(data.DepthMask);
                else
                    v = data.Depth(isfinite(data.Depth) & data.Depth > 0);
                end

                if ~isempty(v)
                    lo = prctile(v, 2);
                    hi = prctile(v, 98);
                    if hi <= lo
                        hi = lo + 1e-3;
                    end
                    set(ax2, 'CLim', [lo hi]);
                end
            end

            title(ax2, sprintf('Depth | Range: [%.2f, %.2f] m | FPS: %.1f', ...
                lo, hi, data.Metrics.DepthFps));
        else
            title(ax2, 'Depth (no data)');
        end

        if mod(frameCount, 30) == 0
            fprintf('Depth (last frame): %.3f s\n', toc(cycleTic));
        end

        drawnow limitrate nocallbacks;

        frameCount = frameCount + 1;

        % Optional: log de tempo de ciclo a cada 30 frames
        if mod(frameCount, 30) == 0
            fprintf('Cycle time (last frame): %.3f s\n', toc(cycleTic));
        end
    end
end

% rDisconnect será chamado automaticamente por cleanupObj

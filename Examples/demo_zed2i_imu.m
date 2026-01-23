% demo_zed2i_imu.m
% IMU stream validation (sensor_msgs/Imu).

clearvars;
close all;
clc;

%% Add project to path (root-based)
PastaAtual = pwd;
PastaRaiz  = 'elt793';

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

%% Create ZED2i object with IMU enabled and guarantee proper cleanup
% Agora a IMU é ativada pela API do construtor (Name-Value).
zed = ZED2i("enableImu", true);

cleanupObj = onCleanup(@() zed.rDisconnect());
zed.rConnect();

t_max = 10;       % duração da demo em segundos
target_fps = 10;  % desired preview FPS
t = tic;
tc = tic;         % timer para controle de FPS

while toc(t) < t_max
    if toc(tc) > 1/target_fps
        tc = tic;

        imu = zed.rGetImu();

        % Se por algum motivo desconectar no meio, aborta de forma limpa
        if ~zed.pFlag.Connected
            fprintf("ZED2i disconnected. LastError: %s\n", string(zed.pFlag.LastError));
            break;
        end

        if zed.pFlag.HasImu
            w = imu.AngularVelocity;
            a = imu.LinearAcceleration;

            fprintf("w=[%.3f %.3f %.3f] rad/s | a=[%.3f %.3f %.3f] m/s^2\n", ...
                w(1), w(2), w(3), a(1), a(2), a(3));
        else
            fprintf("IMU not available. LastError: %s\n", string(zed.pFlag.LastError));
        end

    end
end

% rDisconnect será chamado automaticamente por cleanupObj

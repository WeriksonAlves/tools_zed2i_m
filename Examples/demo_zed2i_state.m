% demo_zed2i_state.m
% IMU + Pose/Odom demo using ZED2i.getSensorData (high-level API).
%
% - Habilita IMU e Pose via construtor (enableImu / enablePose)
% - Usa getSensorData para obter:
%     * Imu (AngularVelocity, LinearAcceleration, etc.)
%     * Pose (Position, OrientationQuat, etc.)
%     * Flags, Metrics e LastError
% - Loga no terminal e acumula trajetória e séries temporais para plot ao final.

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

%% Create ZED2i object with IMU and Pose enabled
zed = ZED2i( ...
    "enableImu",  true, ...
    "enablePose", true ...
);

cleanupObj = onCleanup(@() zed.rosDisconnect());
zed.rosConnect();

%% Buffers para logs de estado
traj          = zeros(0, 3);  % trajetória (XYZ) da pose
tPose         = [];           % timestamps da pose
oriEuler      = [];           % [N x 3] Euler [roll pitch yaw] em rad

tImu          = [];           % timestamps da IMU
angVelImu     = [];           % [N x 3] velocidades angulares IMU
linAccImu     = [];           % [N x 3] acelerações lineares IMU

%% Demo parameters
t_max      = 30;   % duração total [s]
target_fps = 15;   % taxa de atualização desejada (logs/seg)
frameCount = 0;

t  = tic;          % timer total
tc = tic;          % timer de pacing (FPS)
t0 = tic;          % base de tempo para séries temporais

%% Main loop
while toc(t) < t_max

    % Controle simples de FPS
    if toc(tc) > 1 / target_fps
        tc = tic;

        cycleTic = tic;

        % Tempo relativo desde o início da demo (para logs)
        ts = toc(t0);

        % -----------------------------------------------------------------
        % Leitura de alto nível: agrupa tudo em uma struct
        % -----------------------------------------------------------------
        data = zed.getSensorData();

        if mod(frameCount, 30) == 0
            fprintf('getSensorData (last frame): %.3f s\n', toc(cycleTic));
        end

        % Se desconectar no meio, aborta a demo de forma limpa
        if ~data.Connected
            fprintf("[ZED2i] Disconnected. LastError: %s\n", string(data.LastError));
            break;
        end

        % -------------------- IMU (sensor_msgs/Imu) --------------------
        if data.HasImu && ~isempty(data.Imu)
            imu = data.Imu;

            w = imu.AngularVelocity;     % [wx wy wz] rad/s
            a = imu.LinearAcceleration;  % [ax ay az] m/s^2

            % Acumula série temporal
            tImu      = [tImu; ts];           %#ok<AGROW>
            angVelImu = [angVelImu; w(:).'];  %#ok<AGROW>
            linAccImu = [linAccImu; a(:).'];  %#ok<AGROW>

            fprintf("IMU  | w=[%.3f %.3f %.3f] rad/s | a=[%.3f %.3f %.3f] m/s^2\n", ...
                w(1), w(2), w(3), a(1), a(2), a(3));
        else
            fprintf("IMU  not available. LastError: %s\n", string(data.LastError));
        end

        if mod(frameCount, 30) == 0
            fprintf('IMU (last frame): %.3f s\n', toc(cycleTic));
        end

        % -------------------- Pose/Odom (nav_msgs/Odometry) --------------------
        if data.HasPose && ~isempty(data.Pose)
            p = data.Pose.Position;         % [x y z] m
            o = data.Pose.OrientationEuler; % [roll pitch yaw] rad

            traj      = [traj; p(:).'];     %#ok<AGROW>
            tPose     = [tPose; ts];        %#ok<AGROW>
            oriEuler  = [oriEuler; o(:).']; %#ok<AGROW>

            fprintf("POSE | pos=[%.3f %.3f %.3f] m | eul=[roll=%.3f pitch=%.3f yaw=%.3f] rad\n", ...
                p(1), p(2), p(3), o(1), o(2), o(3));
        else
            fprintf("POSE not available. LastError: %s\n", string(data.LastError));
        end

        if mod(frameCount, 30) == 0
            fprintf('Pose (last frame): %.3f s\n', toc(cycleTic));
        end

        frameCount = frameCount + 1;
        fprintf("-----\n");
    end
end

%% Plot trajectory (if pose data was received)
if ~isempty(traj)
    figure('Name', 'ZED2i Trajectory (XYZ)', 'NumberTitle', 'off');
    plot3(traj(:,1), traj(:,2), traj(:,3), '-o');
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Trajectory (XYZ) from /odom');
else
    fprintf("No pose samples collected. Trajectory will not be plotted.\n");
end

%% Plot: Orientação Euler (Pose)
if ~isempty(oriEuler)
    figure('Name', 'Orientation Euler (Pose)', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(tPose, oriEuler(:,1), '-');
    grid on;
    ylabel('roll (rad)');
    title('Orientation - Roll (X)');

    subplot(3,1,2);
    plot(tPose, oriEuler(:,2), '-');
    grid on;
    ylabel('pitch (rad)');
    title('Orientation - Pitch (Y)');

    subplot(3,1,3);
    plot(tPose, oriEuler(:,3), '-');
    grid on;
    ylabel('yaw (rad)');
    xlabel('Time (s)');
    title('Orientation - Yaw (Z)');
else
    fprintf("No orientation Euler samples (Pose) collected.\n");
end

%% Plot: Velocidade Angular (IMU)
if ~isempty(angVelImu)
    figure('Name', 'Angular Velocity (IMU)', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(tImu, angVelImu(:,1), '-');
    grid on;
    ylabel('\omega_x (rad/s)');
    title('Angular Velocity - X');

    subplot(3,1,2);
    plot(tImu, angVelImu(:,2), '-');
    grid on;
    ylabel('\omega_y (rad/s)');
    title('Angular Velocity - Y');

    subplot(3,1,3);
    plot(tImu, angVelImu(:,3), '-');
    grid on;
    ylabel('\omega_z (rad/s)');
    xlabel('Time (s)');
    title('Angular Velocity - Z');
else
    fprintf("No angular velocity samples (IMU) collected.\n");
end

%% Plot: Aceleração Linear (IMU)
if ~isempty(linAccImu)
    figure('Name', 'Linear Acceleration (IMU)', 'NumberTitle', 'off');

    subplot(3,1,1);
    plot(tImu, linAccImu(:,1), '-');
    grid on;
    ylabel('a_x (m/s^2)');
    title('Linear Acceleration - X');

    subplot(3,1,2);
    plot(tImu, linAccImu(:,2), '-');
    grid on;
    ylabel('a_y (m/s^2)');
    title('Linear Acceleration - Y');

    subplot(3,1,3);
    plot(tImu, linAccImu(:,3), '-');
    grid on;
    ylabel('a_z (m/s^2)');
    xlabel('Time (s)');
    title('Linear Acceleration - Z');
else
    fprintf("No linear acceleration samples (IMU) collected.\n");
end

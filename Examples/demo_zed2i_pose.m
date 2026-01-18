% demo_zed2i_pose.m
% Pose/Odom validation (nav_msgs/Odometry).

clearvars;
close all;
clc;

zed = ZED2i("enablePose", true);
cleanupObj = onCleanup(@() zed.rDisconnect());

zed.rConnect();

t_max = 20;
t = tic;

traj = zeros(0, 3);

while toc(t) < t_max
    pose = zed.rGetPose();

    if zed.pFlag.HasPose
        traj(end+1, :) = pose.Position; %#ok<AGROW>
        fprintf("pos=[%.3f %.3f %.3f] m | v=[%.3f %.3f %.3f] m/s\n", ...
            pose.Position(1), pose.Position(2), pose.Position(3), ...
            pose.LinearVelocity(1), pose.LinearVelocity(2), pose.LinearVelocity(3));
    else
        fprintf("Pose not available. LastError: %s\n", string(zed.pFlag.LastError));
    end

    pause(0.05);
end

figure('Name', 'ZED2i Trajectory (XY)', 'NumberTitle', 'off');
plot3(traj(:,1), traj(:,2), traj(:,3), '-o');
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Trajectory (XYZ) from /odom');

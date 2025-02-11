close all

% Run the file to get crane reach workspace data
if exist('crane_reach','var') ~= 1
    crane_reachable_workspace
end

% Run the file to get crane full workspace data
if exist('crane_ws','var') ~= 1
    crane_full_workspace
end

% Run the file to get sawyer workspace data
if exist('nbv_ws','var') ~= 1
    sawyer_full_workspace
end
if exist('nbv_reach','var') ~= 1
    sawyer_reachable_workspace
end

% Import da robits from the URDF file
nbv = importNBV;
rover = importRover("sawyer");

% set the current config as the zero config for da robits
nbvConfig = homeConfiguration(nbv);
roverConfig = homeConfiguration(rover);

% Below is redundant for copy-past purposes
nbvConfig(1).JointPosition = 0;
nbvConfig(2).JointPosition = 0;
nbvConfig(3).JointPosition = 0;
nbvConfig(4).JointPosition = 0;
nbvConfig(5).JointPosition = 0;
nbvConfig(6).JointPosition = 0;

% sawyerConfig = homeConfiguration(sawyer);

nbvConfig(2).JointPosition = pi;
nbvConfig(3).JointPosition = pi/2;

% Show some plots
% 3D YAW PLOT
show(nbv, nbvConfig)
hold on
fill3(nbv_reach.yaw.cartx, nbv_reach.yaw.carty, nbv_reach.yaw.cartz, "r", "FaceAlpha","0.5");
title("NBV Workspace Reach Along Global Z-Axis [3D]")
xlabel("x-reach (meters)")
ylabel("y-reach (meters)")
zlabel("z-reach (meters)")
grid on
axis equal

% 2D YAW PLOT
figure
fill(nbv_reach.yaw.cartx, nbv_reach.yaw.carty, "r", "FaceAlpha","0.5");
hold on
show(nbv, nbvConfig)
title("NBV Workspace Reach Along Global Z-Axis [2D]")
xlabel("x-reach (meters)")
ylabel("y-reach (meters)")
grid on
axis equal


nbvConfig(1).JointPosition = 0;
nbvConfig(2).JointPosition = pi+deg2rad(25);
nbvConfig(3).JointPosition = pi/2;
nbvConfig(4).JointPosition = 0;
nbvConfig(5).JointPosition = 0;
nbvConfig(6).JointPosition = 0;
% 3D PITCH PLOT
figure
show(nbv, nbvConfig)
hold on
fill3(nbv_reach.pitch.cartx, nbv_reach.pitch.carty, nbv_reach.pitch.cartz, "r", "FaceAlpha","0.5");
title("NBV Workspace Reach Along Global Y-Axis [3D]")
xlabel("x-reach (meters)")
ylabel("y-reach (meters)")
zlabel("z-reach (meters)")
grid on
axis equal

% 2D PITCH PLOT
fig = figure;
show(nbv, nbvConfig)
hold on
fill3(nbv_reach.pitch.cartx, nbv_reach.pitch.carty, nbv_reach.pitch.cartz, "r", "FaceAlpha","0.5");
title("NBV Workspace Reach Along Global Y-Axis [2D]")
view([0,1,0])
camproj("orthographic")

xlabel("x-reach (meters)")
zlabel("z-reach (meters)")
xlim([-1.5, 1.5])
zlim([-1,2])
grid on

nbvConfig(1).JointPosition = 0;
nbvConfig(2).JointPosition = pi+deg2rad(25);
nbvConfig(3).JointPosition = -deg2rad(10);
nbvConfig(4).JointPosition = 0;
nbvConfig(5).JointPosition = 0;
nbvConfig(6).JointPosition = 0;
% 3D PITCH PLOT
figure
show(nbv, nbvConfig)
hold on
fill3(nbv_reach.pitch.cartx, nbv_reach.pitch.carty, nbv_reach.pitch.cartz, "r", "FaceAlpha","0.5");
fill3(nbv_reach.elbow.cartx, nbv_reach.elbow.carty, nbv_reach.elbow.cartz, "g", "FaceAlpha","0.5");
title("NBV Workspace Reach Along Global Y-Axis [3D]")
xlabel("x-reach (meters)")
ylabel("y-reach (meters)")
zlabel("z-reach (meters)")
grid on
axis equal

% % 2D PITCH PLOT
% fig = figure;
% show(nbv, nbvConfig)
% hold on
% fill3(nbv_reach.pitch.cartx, nbv_reach.pitch.carty, nbv_reach.pitch.cartz, "r", "FaceAlpha","0.5");
% title("NBV Workspace Reach Along Global Y-Axis [2D]")
% view([0,1,0])
% camproj("orthographic")
% 
% xlabel("x-reach (meters)")
% zlabel("z-reach (meters)")
% xlim([-1.5, 1.5])
% zlim([-1,2])
% grid on
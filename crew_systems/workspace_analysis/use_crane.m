close all

% Run the file to get crane workspace data
if exist('crane_ws','var') ~= 1
    crane_full_workspace
end
if exist('crane_reach_old1','var') ~= 1
    crane_reachable_workspace
end

% Import da robits from the URDF file
crane = importRover("old1");

% set the current config as the zero config for da robits
craneConfig = homeConfiguration(crane);

% Below is redundant for copy-past purposes
craneConfig(1).JointPosition = 0;
craneConfig(2).JointPosition = 0;
craneConfig(3).JointPosition = 0;

% Show some plots
% 3D YAW PLOT
craneConfig(1).JointPosition = deg2rad(10);
craneConfig(2).JointPosition = -pi/2;
craneConfig(3).JointPosition = 1.5;
show(crane, craneConfig);
hold on
fill3(crane_reach_old1.yaw.cartx, crane_reach_old1.yaw.carty, crane_reach_old1.yaw.cartz, "r", "FaceAlpha","0.5");
title("Rover Crane Workspace Reach Along Global Z-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D YAW PLOT
figure;
fill(crane_reach_old1.yaw.cartx, crane_reach_old1.yaw.carty, "r", "FaceAlpha","0.5");
hold on
show(crane, craneConfig);
title("Rover Crane Workspace Reach Along Global Z-Axis [2D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
grid on
axis equal


% 3D PITCH PLOT
craneConfig(1).JointPosition = 0;
craneConfig(2).JointPosition = -pi/4;
craneConfig(3).JointPosition = 1.5;

figure;
show(crane, craneConfig);
hold on
fill3(crane_reach_old1.pitch.cartx, crane_reach_old1.pitch.carty, crane_reach_old1.pitch.cartz, "r", "FaceAlpha","0.5");
title("Rover Crane Workspace Reach Along Global Y-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D PITCH PLOT
figure
show(crane, craneConfig);
hold on
fill3(crane_reach_old1.pitch.cartx, crane_reach_old1.pitch.carty, crane_reach_old1.pitch.cartz, "r", "FaceAlpha","0.5");
title("Rover Crane Workspace Reach Along Global Y-Axis [2D]");
view([0,1,0]);
camproj("orthographic");
xlabel("x-reach (meters)");
zlabel("z-reach (meters)");
xlim([-5,2]);
zlim([0, 8]);
grid on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% now do the new rover lol
% Import da robits from the URDF file
% crane = importRover("crane_old2");
crane = importRover("old2");

% set the current config as the zero config for da robits
craneConfig = homeConfiguration(crane);
% cranePosition = [1.1,0,0.5,0];
cranePosition = [0,0,0,0];

% Below is redundant for copy-past purposes
craneConfig(1).JointPosition = 0;
% craneConfig(2).JointPosition = 0;
% craneConfig(3).JointPosition = 0;

% Show some plots
% 3D YAW PLOT
craneConfig(1).JointPosition = deg2rad(10);
craneConfig(2).JointPosition = -pi/2;
craneConfig(3).JointPosition = 1.2267;
craneConfig(4).JointPosition = 1.2267;
craneConfig(5).JointPosition = 1.2267;
figure
show(crane, craneConfig, Position=cranePosition);
hold on
fill3(crane_reach_old2.yaw.cartx, crane_reach_old2.yaw.carty, crane_reach_old2.yaw.cartz, "r", "FaceAlpha","0.5");
fill3(crane_reach_old2.yaw_rigid.cartx, crane_reach_old2.yaw_rigid.carty, crane_reach_old2.yaw_rigid.cartz, "g", "FaceAlpha","0.5");
title("Rover Crane Workspace Reach Along Global Z-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D YAW PLOT
figure;
fill(crane_reach_old2.yaw.cartx, crane_reach_old2.yaw.carty, "r", "FaceAlpha","0.5");
fill(crane_reach_old2.yaw_rigid.cartx, crane_reach_old2.yaw_rigid.carty, "g", "FaceAlpha","0.5");
hold on
show(crane, craneConfig, Position=cranePosition);
title("Rover Crane Workspace Reach Along Global Z-Axis [2D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
grid on
axis equal


% 3D PITCH PLOT
craneConfig(1).JointPosition = 0;
craneConfig(2).JointPosition = -pi/4;
craneConfig(3).JointPosition = 1.2267;
craneConfig(4).JointPosition = 1.2267;
craneConfig(5).JointPosition = 1.2267;


figure;
show(crane, craneConfig, Position=cranePosition);
hold on
fill3(crane_reach_old2.pitch.cartx, crane_reach_old2.pitch.carty, crane_reach_old2.pitch.cartz, "r", "FaceAlpha","0.5");
fill3(crane_reach_old2.pitch_rigid.cartx, crane_reach_old2.pitch_rigid.carty, crane_reach_old2.pitch_rigid.cartz, "g", "FaceAlpha","0.5");

title("Rover Crane Workspace Reach Along Global Y-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D PITCH PLOT
figure
show(crane, craneConfig, Position=cranePosition);
hold on
fill3(crane_reach_old2.pitch.cartx, crane_reach_old2.pitch.carty, crane_reach_old2.pitch.cartz, "r", "FaceAlpha","0.5");
fill3(crane_reach_old2.pitch_rigid.cartx, crane_reach_old2.pitch_rigid.carty, crane_reach_old2.pitch_rigid.cartz, "r", "FaceAlpha","0.5");
title("Rover Crane Workspace Reach Along Global Y-Axis [2D]");
view([0,1,0]);
camproj("orthographic");
xlabel("x-reach (meters)");
zlabel("z-reach (meters)");
xlim([-5,2]);
zlim([0, 8]);
grid on








% % Full workspace plot
% count = 1;
% sample_interval = floor(length(crane_ws.full_unique.cartx)/10000);
% x_sample = zeros(sample_interval, 1);
% y_sample = zeros(sample_interval, 1);
% z_sample = zeros(sample_interval, 1);
% for i = 1:sample_interval:length(crane_ws.full_unique.cartx)
%     x_sample(count) = crane_ws.full_unique.cartx(i);
%     y_sample(count) = crane_ws.full_unique.carty(i);
%     z_sample(count) = crane_ws.full_unique.cartz(i);
%     count = count + 1;
% end
% 
% figure
% % Reset crane config
% craneConfig(1).JointPosition = 0;
% craneConfig(2).JointPosition = 0;
% craneConfig(3).JointPosition = 0;
% craneConfig(4).JointPosition = 0;
% craneConfig(5).JointPosition = 0;
% craneConfig(6).JointPosition = 0;
% craneConfig(7).JointPosition = 0;
% 
% show(crane, craneConfig);
% hold on
% scatter3(x_sample,y_sample,z_sample, 0.25, ".b");
% title("Subset of Unique Cartesian Positions in crane Workspace")
% xlabel("Cartesian X (meters)")
% ylabel("Cartesian Y (meters)")
% zlabel("Cartesian Z (meters)")
% xlim([-2,2]);
% ylim([-2,2]);
% zlim([-3,3]);
% axis equal
% 
% figure
% [values, centers] = hist3([transpose(crane_ws.full.cartx), transpose(crane_ws.full.carty)],[56 56]);
% imagesc(centers{:}, values.')
% c = colorbar;
% xlabel(c, "Number of Repeated Positions")
% title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
% xlabel("Cartesian X (meters)")
% ylabel("Cartesian Y (meters)")
% axis equal
% axis xy
% hold on
% show(crane, craneConfig);
% 
% figure
% [values, centers] = hist3([transpose(crane_ws.full.cartx), transpose(crane_ws.full.cartz)],[56 56]);
% imagesc(centers{:}, values.')
% c = colorbar;
% xlabel(c, "Number of Repeated Positions")
% title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
% xlabel("Cartesian X (meters)")
% ylabel("Cartesian Z (meters)")
% axis equal
% axis xy
% hold on
% craneConfig(1).JointPosition = pi;
% craneConfig(2).JointPosition = 0;
% craneConfig(3).JointPosition = 0;
% craneConfig(4).JointPosition = 0;
% craneConfig(5).JointPosition = 0;
% craneConfig(6).JointPosition = 0;
% craneConfig(7).JointPosition = 0;
% show(crane_xz, craneConfig);
% 
% figure
% [values, centers] = hist3([transpose(crane_ws.full.carty), transpose(crane_ws.full.cartz)],[56 56]);
% imagesc(centers{:}, values.')
% c = colorbar;
% xlabel(c, "Number of Repeated Positions")
% title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
% xlabel("Cartesian Y (meters)")
% ylabel("Cartesian Z (meters)")
% axis equal
% axis xy
% hold on
% craneConfig(1).JointPosition = -pi/2;
% craneConfig(2).JointPosition = 0;
% craneConfig(3).JointPosition = 0;
% craneConfig(4).JointPosition = 0;
% craneConfig(5).JointPosition = 0;
% craneConfig(6).JointPosition = 0;
% craneConfig(7).JointPosition = 0;
% show(crane_xz, craneConfig);
% 
% figure
% count = 1;
% for i = 1:sample_interval:length(crane_ws.full.q(1,:))
%     hold off
%     craneConfig(1).JointPosition = crane_ws.full.q(1,i);
%     craneConfig(2).JointPosition = crane_ws.full.q(2,i);
%     craneConfig(3).JointPosition = crane_ws.full.q(3,i);
%     craneConfig(4).JointPosition = crane_ws.full.q(4,i);
%     craneConfig(5).JointPosition = crane_ws.full.q(5,i);
%     craneConfig(6).JointPosition = crane_ws.full.q(6,i);
%     craneConfig(6).JointPosition = crane_ws.full.q(7,i);
%     show(crane, craneConfig);
%     hold on
%     scatter3(x_sample, y_sample, z_sample, 0.25, ".r");
%     count = count + 1;
%     drawnow limitrate
% end




clear i density samezies1 samezies2 samezies3 samezies4 samezies count tol_err_ant x_sample y_sample z_sample c values centers fig sample_interval crane_yz



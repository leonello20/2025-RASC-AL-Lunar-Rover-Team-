close all

% Run the file to get dyma workspace data
if exist('dyma_ws','var') ~= 1
    dymaflight_full_workspace
end
if exist('dyma_reach','var') ~= 1
    dymaflight_reachable_workspace
end

% Import da robits from the URDF file
dyma = importDyma("");
dyma_xz = importDyma("xz");

% set the current config as the zero config for da robits
dymaConfig = homeConfiguration(dyma);

% Below is redundant for copy-past purposes
dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = 0;
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;

% Show some plots
% 3D YAW PLOT
dymaConfig(2).JointPosition = pi/2;
show(dyma, dymaConfig);
hold on
fill3(dyma_reach.yaw.cartx, dyma_reach.yaw.carty, dyma_reach.yaw.cartz, "r", "FaceAlpha","0.5");
title("Dymaflight Workspace Reach Along Global Z-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D YAW PLOT
figure;
fill(dyma_reach.yaw.cartx, dyma_reach.yaw.carty, "r", "FaceAlpha","0.5");
hold on
show(dyma, dymaConfig);
title("Dymaflight Workspace Reach Along Global Z-Axis [2D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
grid on
axis equal


% 3D PITCH PLOT
dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = deg2rad(120);
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;
figure;
show(dyma, dymaConfig);
hold on
fill3(dyma_reach.pitch.cartx, dyma_reach.pitch.carty, dyma_reach.pitch.cartz, "r", "FaceAlpha","0.5");
title("Dymaflight Workspace Reach Along Global Y-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal

% 2D PITCH PLOT
figure;
show(dyma, dymaConfig);
hold on
fill3(dyma_reach.pitch.cartx, dyma_reach.pitch.carty, dyma_reach.pitch.cartz, "r", "FaceAlpha","0.5");
title("Dymaflight Workspace Reach Along Global Y-Axis [2D]");
view([0,1,0]);
camproj("orthographic");
xlabel("x-reach (meters)");
zlabel("z-reach (meters)");
xlim([-1.5, 1.5]);
zlim([-1, 2]);
grid on

% 3D Elbow plot
figure
dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = deg2rad(120);
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = deg2rad(155);
dymaConfig(5).JointPosition = pi;
dymaConfig(6).JointPosition = deg2rad(-65);
dymaConfig(7).JointPosition = 0;
show(dyma, dymaConfig);
hold on
fill3(dyma_reach.pitch.cartx, dyma_reach.pitch.carty, dyma_reach.pitch.cartz, "r", "FaceAlpha","0.5");
fill3(dyma_reach.elbow.cartx, dyma_reach.elbow.carty, dyma_reach.elbow.cartz, "g", "FaceAlpha","0.5");
fill3(dyma_reach.wrist.cartx, dyma_reach.wrist.carty, dyma_reach.wrist.cartz, "b", "FaceAlpha","0.5");
title("Dymaflight Workspace Reach Along Global Y-Axis [3D]");
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");
zlabel("z-reach (meters)");
grid on
axis equal


% Full workspace plot
count = 1;
sample_interval = floor(length(dyma_ws.full_unique.cartx)/15000);
x_sample = zeros(sample_interval, 1);
y_sample = zeros(sample_interval, 1);
z_sample = zeros(sample_interval, 1);
for i = 1:sample_interval:length(dyma_ws.full_unique.cartx)
    x_sample(count) = dyma_ws.full_unique.cartx(i);
    y_sample(count) = dyma_ws.full_unique.carty(i);
    z_sample(count) = dyma_ws.full_unique.cartz(i);
    count = count + 1;
end

figure
% Reset dyma config
dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = 0;
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;

show(dyma, dymaConfig);
hold on
scatter3(x_sample,y_sample,z_sample, 0.25, ".b");
title("Subset of Unique Cartesian Positions in Dyma Workspace")
xlabel("Cartesian X (meters)")
ylabel("Cartesian Y (meters)")
zlabel("Cartesian Z (meters)")
xlim([-2,2]);
ylim([-2,2]);
zlim([-3,3]);
axis equal


dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = pi/2;
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;
figure
[values, centers] = hist3([transpose(dyma_ws.full.cartx), transpose(dyma_ws.full.carty)],[51, 51]);
imagesc(centers{:}, values.')
c = colorbar;
xlabel(c, "Number of Repeated Positions")
title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
xlabel("Cartesian X (meters)")
ylabel("Cartesian Y (meters)")
axis equal
axis xy
hold on
show(dyma, dymaConfig);

dymaConfig(1).JointPosition = 0;
dymaConfig(2).JointPosition = pi/2;
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;
figure
[values, centers] = hist3([transpose(dyma_ws.full.cartx), transpose(dyma_ws.full.cartz)],[51, 51]);
imagesc(centers{:}, values.')
c = colorbar;
xlabel(c, "Number of Repeated Positions")
title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
xlabel("Cartesian X (meters)")
ylabel("Cartesian Z (meters)")
axis equal
axis xy
hold on
show(dyma_xz, dymaConfig);

dymaConfig(1).JointPosition = pi/2;
dymaConfig(2).JointPosition = pi/2;
dymaConfig(3).JointPosition = 0;
dymaConfig(4).JointPosition = 0;
dymaConfig(5).JointPosition = 0;
dymaConfig(6).JointPosition = 0;
dymaConfig(7).JointPosition = 0;
figure
[values, centers] = hist3([transpose(dyma_ws.full.carty), transpose(dyma_ws.full.cartz)],[51, 51]);
imagesc(centers{:}, values.')
c = colorbar;
xlabel(c, "Number of Repeated Positions")
title("Density Plot of Repeated Cartesian Positions With Different Joint Configuration")
xlabel("Cartesian Y (meters)")
ylabel("Cartesian Z (meters)")
axis equal
axis xy
hold on
show(dyma_xz, dymaConfig);

% % figure
% % count = 1;
% % for i = 1:sample_interval:length(dyma_ws.full.q(1,:))
% %     hold off
% %     dymaConfig(1).JointPosition = dyma_ws.full.q(1,i);
% %     dymaConfig(2).JointPosition = dyma_ws.full.q(2,i);
% %     dymaConfig(3).JointPosition = dyma_ws.full.q(3,i);
% %     dymaConfig(4).JointPosition = dyma_ws.full.q(4,i);
% %     dymaConfig(5).JointPosition = dyma_ws.full.q(5,i);
% %     dymaConfig(6).JointPosition = dyma_ws.full.q(6,i);
% %     dymaConfig(6).JointPosition = dyma_ws.full.q(7,i);
% %     show(dyma, dymaConfig);
% %     hold on
% %     scatter3(x_sample, y_sample, z_sample, 0.25, ".r");
% %     count = count + 1;
% %     drawnow limitrate
% % end




clear i density samezies1 samezies2 samezies3 samezies4 samezies count tol_err_ant x_sample y_sample z_sample c values centers fig sample_interval dyma_yz




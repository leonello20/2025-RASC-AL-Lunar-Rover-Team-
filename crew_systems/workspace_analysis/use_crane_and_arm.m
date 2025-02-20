close all
addpath("kinematics\")

craneDH = [
    0            0              0.3             0;
    0           -pi/2           0.0             0;
    0            pi/2           1.4             0;
    0           -pi/2           0               0;
   -1.054        0              0               0];
crane_cart_offset = [1.1, 0, 0.5]; % cartesian offset

craneDH_mod = craneDH;

% Import da robits from the URDF file
crane = importRover("dyma");

% set the current config as the zero config for da robits
craneConfig = homeConfiguration(crane);
craneConfig(1).JointPosition = 0;
craneConfig(2).JointPosition = -pi/2.75;
craneConfig(3).JointPosition = 0;
craneConfig(4).JointPosition = 0.8;
craneConfig(5).JointPosition = 0.8;
craneConfig(6).JointPosition = deg2rad(35);

craneConfig(7).JointPosition = deg2rad(0);
craneConfig(8).JointPosition = deg2rad(45);
% craneConfig(9).JointPosition = deg2rad(-35);
% craneConfig(10).JointPosition = deg2rad(50);
% craneConfig(11).JointPosition = deg2rad(180);
% craneConfig(12).JointPosition = deg2rad(45);

show(crane, craneConfig);
hold on
fill(crane_reach.yaw.cartx, crane_reach.yaw.carty, "r", "FaceAlpha","0.5")
xlabel("x-reach (meters)");
ylabel("y-reach (meters)");

grid on

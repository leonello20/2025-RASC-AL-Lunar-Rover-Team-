close all
clear

% Run the file to get sawyer workspace data
sawyer_reachable_workspace

% Import da robits from the URDF file
sawyer = importSawyer;
robot = rigidBodyTree;
bodies = cell(7,1);
joints = cell(7,1);
for i = 1:7
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},sawyerDH(i,:),"dh"); % USING STANDARD DH PARAMETERS
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end

% set the current config as the zero config for da robits
sawyerConfig = homeConfiguration(sawyer);
robotConfig = homeConfiguration(robot);

% Below is redundant for copy-past purposes
sawyerConfig(1).JointPosition = 0;
sawyerConfig(2).JointPosition = pi/2;
sawyerConfig(3).JointPosition = pi;
sawyerConfig(4).JointPosition = pi;
sawyerConfig(5).JointPosition = 0;
sawyerConfig(6).JointPosition = 0;
sawyerConfig(7).JointPosition = 0;

show(sawyer, sawyerConfig)
hold on
show(robot, robotConfig)


% sawyerConfig(2).JointPosition = pi;
% sawyerConfig(3).JointPosition = pi/2;
% 
% % Show some plots
% % 3D YAW PLOT
% show(sawyer, sawyerConfig)
% hold on
% fill3(sawyer_reach.yaw.cartx, sawyer_reach.yaw.carty, sawyer_reach.yaw.cartz, "r", "FaceAlpha","0.5");
% title("sawyer Workspace Reach Along Global Z-Axis [3D]")
% xlabel("x-reach (meters)")
% ylabel("y-reach (meters)")
% zlabel("z-reach (meters)")
% grid on
% axis equal
% 
% % 2D YAW PLOT
% figure
% fill(sawyer_reach.yaw.cartx, sawyer_reach.yaw.carty, "r", "FaceAlpha","0.5");
% hold on
% show(sawyer, sawyerConfig)
% title("sawyer Workspace Reach Along Global Z-Axis [2D]")
% xlabel("x-reach (meters)")
% ylabel("y-reach (meters)")
% grid on
% axis equal
% 
% 
% sawyerConfig(1).JointPosition = 0;
% sawyerConfig(2).JointPosition = pi+deg2rad(25);
% sawyerConfig(3).JointPosition = pi/2;
% sawyerConfig(4).JointPosition = 0;
% sawyerConfig(5).JointPosition = 0;
% sawyerConfig(6).JointPosition = 0;
% % 3D PITCH PLOT
% figure
% show(sawyer, sawyerConfig)
% hold on
% fill3(sawyer_reach.pitch.cartx, sawyer_reach.pitch.carty, sawyer_reach.pitch.cartz, "r", "FaceAlpha","0.5");
% title("sawyer Workspace Reach Along Global Y-Axis [3D]")
% xlabel("x-reach (meters)")
% ylabel("y-reach (meters)")
% zlabel("z-reach (meters)")
% grid on
% axis equal
% 
% % 2D PITCH PLOT
% fig = figure;
% show(sawyer, sawyerConfig)
% hold on
% fill3(sawyer_reach.pitch.cartx, sawyer_reach.pitch.carty, sawyer_reach.pitch.cartz, "r", "FaceAlpha","0.5");
% title("sawyer Workspace Reach Along Global Y-Axis [2D]")
% view([0,1,0])
% camproj("orthographic")
% 
% xlabel("x-reach (meters)")
% zlabel("z-reach (meters)")
% xlim([-1.5, 1.5])
% zlim([-1,2])
% grid on
% 
% sawyerConfig(1).JointPosition = 0;
% sawyerConfig(2).JointPosition = pi+deg2rad(25);
% sawyerConfig(3).JointPosition = -deg2rad(10);
% sawyerConfig(4).JointPosition = 0;
% sawyerConfig(5).JointPosition = 0;
% sawyerConfig(6).JointPosition = 0;
% % 3D PITCH PLOT
% figure
% show(sawyer, sawyerConfig)
% hold on
% fill3(sawyer_reach.pitch.cartx, sawyer_reach.pitch.carty, sawyer_reach.pitch.cartz, "r", "FaceAlpha","0.5");
% fill3(sawyer_reach.elbow.cartx, sawyer_reach.elbow.carty, sawyer_reach.elbow.cartz, "g", "FaceAlpha","0.5");
% title("sawyer Workspace Reach Along Global Y-Axis [3D]")
% xlabel("x-reach (meters)")
% ylabel("y-reach (meters)")
% zlabel("z-reach (meters)")
% grid on
% axis equal

% % 2D PITCH PLOT
% fig = figure;
% show(sawyer, sawyerConfig)
% hold on
% fill3(sawyer_reach.pitch.cartx, sawyer_reach.pitch.carty, sawyer_reach.pitch.cartz, "r", "FaceAlpha","0.5");
% title("sawyer Workspace Reach Along Global Y-Axis [2D]")
% view([0,1,0])
% camproj("orthographic")
% 
% xlabel("x-reach (meters)")
% zlabel("z-reach (meters)")
% xlim([-1.5, 1.5])
% zlim([-1,2])
% grid on
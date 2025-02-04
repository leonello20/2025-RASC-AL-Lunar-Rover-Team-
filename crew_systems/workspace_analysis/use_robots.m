close all
clear

nbv_reachable_workspace
load('sawyer_workspace_data.mat')

robot = rigidBodyTree;
bodies = cell(6,1);
joints = cell(6,1);
for i = 1:6
    bodies{i} = rigidBody(['body' num2str(i)]);
    joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
    setFixedTransform(joints{i},nbvDH(i,:),"mdh");
    bodies{i}.Joint = joints{i};
    if i == 1 % Add first body to base
        addBody(robot,bodies{i},"base")
    else % Add current body to previous body by name
        addBody(robot,bodies{i},bodies{i-1}.Name)
    end
end



nbv = importNBV;
sawyer = importSawyer;

config = homeConfiguration(robot);

nbvConfig = homeConfiguration(nbv);
sawyerConfig = homeConfiguration(sawyer);

j2 = 0;
j3 = 0;

config(2).JointPosition = j2;
nbvConfig(2).JointPosition = j2;

config(3).JointPosition = j3;
nbvConfig(3).JointPosition = j3;


show(nbv, nbvConfig)
hold on
show(robot, config)
show(sawyer)
plot3(yaw_plane_cartx, yaw_plane_carty, yaw_plane_cartz);
axis equal

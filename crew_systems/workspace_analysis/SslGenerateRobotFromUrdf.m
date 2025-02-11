function robot_out = SslGenerateRobotFromUrdf(dh_table, dh_style)
    robot = rigidBodyTree;
    num_rows = length(dh_table(:,1));
    bodies = cell(num_rows,1);
    joints = cell(num_rows,1);
    if(dh_style == "modified")
        style = "mdh";
    elseif(dh_style == "standard")
        style = "dh";
    else
        error("Options are 'modified' or 'standard'")
    end
    for i = 1:num_rows
        bodies{i} = rigidBody(['body' num2str(i)]);
        joints{i} = rigidBodyJoint(['jnt' num2str(i)],"revolute");
        setFixedTransform(joints{i},dh_table(i,:),style);
        bodies{i}.Joint = joints{i};
        if i == 1 % Add first body to base
            addBody(robot,bodies{i},"base")
        else % Add current body to previous body by name
            addBody(robot,bodies{i},bodies{i-1}.Name)
        end
    end
    robot_out = robot;
end
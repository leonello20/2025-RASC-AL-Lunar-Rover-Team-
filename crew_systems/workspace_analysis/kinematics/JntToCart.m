
function p = JntToCart(dh_table, dh_style, joint_array, is_relative)
    % Get some lengths
    num_joints = length(joint_array); % get number of joints
    num_rows = length(dh_table(:,1)); % get number of rows in DH table

    % Indicate if we have a tool attached
    is_tool = false;
    if num_joints + 1 == num_rows
        warning("Number of DH table rows is +1 more than number of supplied joints, assmuming final row is Tool Tip position");
        is_tool = true;
    elseif num_joints ~= num_rows
        error("Number of supplied joints is different from number of rows in the DH table!")
    end

    % Input values into the DH table
    for i = 1:num_joints
        if(is_relative == true)
            dh_table(i,4) = dh_table(i,4) + joint_array(i); % set the theta value of the dh table to be the current value + the input relative joint position
        elseif(is_relative == false)
            dh_table(i,4) = joint_array(i); % set the theta value of the dh table to be the inputted joint position
        end
    end
    

    if(dh_style == "modified")
        if(is_tool == false)
            T_final = SslModifDhTableToTransf(0, num_joints, dh_table);
        elseif(is_tool == true)
            T_final = SslModifDhTableToTransf(0, num_joints+1, dh_table);
        end
    elseif(dh_style == "standard")
        if(is_tool == false)
            T_final = SslStandardDhTableToTransf(0, num_joints, dh_table);
        elseif(is_tool == true)
            T_final = SslStandardDhTableToTransf(0, num_joints+1, dh_table);
        end
    else
        error("dh_style options are 'modified' and 'standard'")
    end
    rot = [T_final(1,1:3); T_final(2,1:3); T_final(3,1:3);];
    rot_vec = rotm2eul(rot,'ZYX');
    R = rot_vec(3);
    P = rot_vec(2);
    Y = rot_vec(1);
    x = T_final(1,4);
    y = T_final(2,4);
    z = T_final(3,4);
    p = [x y z R P Y];
    

end
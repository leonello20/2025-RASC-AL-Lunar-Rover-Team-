% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %


iter_per_joint_array = [1,180,180,180,180,180,180];
% Z is "HEIGHT"
% X is "LENGTH"
% Y is "WIDTH"


% ROUGH NUMBERS - NOT FINAL 
chassis_length = 2.4384; % meters
chassis_height = 1 % meters
chassis_width = 1.3716 % meters

chassis_point_1 = [0,0,0] % front-top, right
chassis_point_2 = [0,0,0] % front-front, right 

% RECALL THAT DH TABLES START i=0 IS THE BASE LINK, i.e [0,0,0]
% units are degrees and meters

% Make a DH table for refernce
% __|       d(i)        theta(i)       a(i-1)      alpha(i-1)
% 1 |       0.25          th1             0           0
% 2 |       0             th2             0           90
% 3 |       0             th3             0.5         0
% .         .             .               .           .
% .         .             .               .           .
% .         .             .               .           .
% j |       0.4           th_j             0          -90   <- EE  


% SAWYER DH table (for reference)
% __|       d(i)        theta(i)       a(i-1)      alpha(i-1)
% 1 |     0.237	          th1           0.081         -90
% 2 |     0.1925          th2           0             -90
% 3 |     0.4             th3           0             -90
% 4 |    -0.1685          th4           0             -90
% 5 |     0.4             th5           0             -90
% 6 |     0.1363          th6           0             -90
% 7 |     0.11            th7           0             -90
% T |     0               thT           0              0

sawyer_DH = [ 
     0.237	         0           0.081         -90
     0.1925          0           0             -90
     0.4             0           0             -90
    -0.1685          0           0             -90
     0.4             0           0             -90
     0.1363          0           0             -90
     0.11            0           0             -90
     0               0           0              0];

sawyer_limits = [   -180    180;
                    0       180;
                    -90     90;
                    -180    180;
                    -90     90;
                    -180    180;
                    -180    180];

% Get the recquired vector length so we don't make it too big
if(length(iter_per_joint_array) ~= 7)
    error("NBVs iter_per_joint_array must be length of 6! must contain desired iterations for each joint")
end

count = 1;
for th1=sawyer_limits(1,1):iter_per_joint_array(1):sawyer_limits(1,2)
    for th2=sawyer_limits(2,1):iter_per_joint_array(2):sawyer_limits(2,2)
        for th3=sawyer_limits(3,1):iter_per_joint_array(3):sawyer_limits(3,2)
            for th4=sawyer_limits(4,1):iter_per_joint_array(4):sawyer_limits(4,2)
                for th5=sawyer_limits(5,1):iter_per_joint_array(5):sawyer_limits(5,2)
                    for th6=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                        for th7=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                            count = count+1;
                        end
                    end
                end
            end
        end
    end
end
data_length = count; % Get the length of the robit arm
fprintf("Array length: " + int2str(data_length) + "\n")
count = 1; % Start a counter
cartx(data_length) = 0; % initialize the cartesian x vector
carty(data_length) = 0; % initialize the cartesian y vector
cartz(data_length) = 0; % initialize the cartesian z vector

% Begin the long ass iteration (its so over)
for th1=sawyer_limits(1,1):iter_per_joint_array(1):sawyer_limits(1,2)
    fprintf("Started interation, current J1 angle is " + int2str(th1) + "\n")
    for th2=sawyer_limits(2,1):iter_per_joint_array(2):sawyer_limits(2,2)
        for th3=sawyer_limits(3,1):iter_per_joint_array(3):sawyer_limits(3,2)
            for th4=sawyer_limits(4,1):iter_per_joint_array(4):sawyer_limits(4,2)
                for th5=sawyer_limits(5,1):iter_per_joint_array(5):sawyer_limits(5,2)
                    for th6=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                        for th7=sawyer_limits(7,1):iter_per_joint_array(7):sawyer_limits(7,2)
                            % Get the DH transformation matricies
                            T1 = RotMat(sawyer_DH(1,4), "x", "deg")*TransMat(sawyer_DH(1,3), "a")*RotMat(th1, "z", "deg")*TransMat(sawyer_DH(1,1), "d");
                            T2 = RotMat(sawyer_DH(2,4), "x", "deg")*TransMat(sawyer_DH(2,3), "a")*RotMat(th2, "z", "deg")*TransMat(sawyer_DH(2,1), "d");
                            T3 = RotMat(sawyer_DH(3,4), "x", "deg")*TransMat(sawyer_DH(3,3), "a")*RotMat(th3, "z", "deg")*TransMat(sawyer_DH(3,1), "d");
                            T4 = RotMat(sawyer_DH(4,4), "x", "deg")*TransMat(sawyer_DH(4,3), "a")*RotMat(th4, "z", "deg")*TransMat(sawyer_DH(4,1), "d");
                            T5 = RotMat(sawyer_DH(5,4), "x", "deg")*TransMat(sawyer_DH(5,3), "a")*RotMat(th5, "z", "deg")*TransMat(sawyer_DH(5,1), "d");
                            T6 = RotMat(sawyer_DH(6,4), "x", "deg")*TransMat(sawyer_DH(6,3), "a")*RotMat(th6, "z", "deg")*TransMat(sawyer_DH(6,1), "d");
                            T7 = RotMat(sawyer_DH(7,4), "x", "deg")*TransMat(sawyer_DH(7,3), "a")*RotMat(sawyer_DH(7,2), "z", "deg")*TransMat(sawyer_DH(7,1), "d");
                            T_final = T1*T2*T3*T4*T5*T6*T7; % get the final transformation matrix
                            cartx(count) = T_final(1,4); % Save the value in the x vector
                            carty(count) = T_final(2,4); % Save the value in the y vector
                            cartz(count) = T_final(3,4); % Save the value in the z vector
                            count = count+1; % iterate up
                        end
                    end
                end
            end
        end
    end
end

% Now save the vector outputs
file_name = "sawyer_workspace_data";
sawyerx = cartx;
sawyery = carty;
sawyerz = cartz;
save(file_name, "sawyerx", "sawyery", "sawyerz", "-v7.3");
% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %

addpath("kinematics\")


% RECALL THAT MODIFIED DH TABLES START i=0 IS THE BASE LINK, i.e [0,0,0]
% units are degrees and meters

% Make a DH table for refernce
% __|      a(i-1),            alpha(i-1),       d(i),         theta(i)
% 1 |       a0,                 alpha0,         d1,           theta1 
% 2 |       a1,                 alpha1,         d2,           theta2;  
%   |       .                     .             .               .   
%   |       .                     .             .               .   
%   |       .                     .             .               .  
% j |       aj-1,               alphaj-1,       dj,           thetaj;  

% SAWYER DH table (for reference)
% __|       a(i-1)      alpha(i-1)      d(i)        theta(i)
% 1 |       0.081       -pi/2           0.237           th1
% 2 |       0           -pi/2           0.1925          th2
% 3 |       0           -pi/2           0.4             th3
% 4 |       0           -pi/2          -0.1685          th4
% 5 |       0           -pi/2           0.4             th5
% 6 |       0           -pi/2           0.1363          th6
% 7 |       0           -pi/2           0.11            th7
% T |       0           0               0               thT



sawyerDH = [ 
    0.081       -pi/2           0.317            0;
    0           -pi/2           0.1925          -pi/2;
    0           -pi/2           0.4             -pi;
    0           -pi/2          -0.1685          -pi;
    0           -pi/2           0.4             -pi;
    0           -pi/2           0.1363          -pi;
    0            0              0.13375         -pi;
    0            0              0                0];

sawyerDH_mod = sawyerDH;


sawyer_limits = [-pi    pi;
                  pi    pi;
                 -pi    pi;
                 -deg2rad(160) deg2rad(160);
                 -pi    pi;
                 -pi    pi;
                 -pi    pi];

iter_per_joint_array = [deg2rad(30),deg2rad(15),deg2rad(15),deg2rad(15),deg2rad(15),deg2rad(15),deg2rad(15)];


% Get the recquired vector length so we don't make it too big
if(length(iter_per_joint_array) ~= 7)
    error("Sawyer's iter_per_joint_array must be length of 7! must contain desired iterations for each joint")
end

count = 1;
for th1=sawyer_limits(1,1):iter_per_joint_array(1):sawyer_limits(1,2)
    for th2=sawyer_limits(2,1):iter_per_joint_array(2):sawyer_limits(2,2)
        for th3=sawyer_limits(3,1):iter_per_joint_array(3):sawyer_limits(3,2)
            for th4=sawyer_limits(4,1):iter_per_joint_array(4):sawyer_limits(4,2)
                for th5=sawyer_limits(5,1):iter_per_joint_array(5):sawyer_limits(5,2)
                    for th6=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                        for th7=sawyer_limits(7,1):iter_per_joint_array(7):sawyer_limits(7,2)
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

% initialize our vectors
sawyer_ws.full.cartx(data_length) = 0; % initialize the sawyer_ws.full.cartesian x vector
sawyer_ws.full.carty(data_length) = 0; % initialize the sawyer_ws.full.cartesian y vector
sawyer_ws.full.cartz(data_length) = 0; % initialize the sawyer_ws.full.cartesian z vector

% reset counter
count = 1; 
% Begin the long ass iteration (its so over)
for th1=sawyer_limits(1,1):iter_per_joint_array(1):sawyer_limits(1,2)
    fprintf("Started interation, current J1 angle is ")
    th1

    % compute the transform T1
    T1 = SslStandardDhToTransf(sawyerDH(1,1), sawyerDH(1,2), sawyerDH(1,3), sawyerDH(1,4)+th1);

    for th2=sawyer_limits(2,1):iter_per_joint_array(2):sawyer_limits(2,2)
        % Compute transform T_1_2
        T_1_2 = T1*SslStandardDhToTransf(sawyerDH(2,1), sawyerDH(2,2), sawyerDH(2,3), sawyerDH(2,4)+th2);

        for th3=sawyer_limits(3,1):iter_per_joint_array(3):sawyer_limits(3,2)
            % Compute transform T_1_3
            T_1_3 = T_1_2 * SslStandardDhToTransf(sawyerDH(3,1), sawyerDH(3,2), sawyerDH(3,3), sawyerDH(3,4)+th3);
            
            for th4=sawyer_limits(4,1):iter_per_joint_array(4):sawyer_limits(4,2)
                % Compute transform T_1_4
                T_1_4 = T_1_3 * SslStandardDhToTransf(sawyerDH(4,1), sawyerDH(4,2), sawyerDH(4,3), sawyerDH(4,4)+th4);

                for th5=sawyer_limits(5,1):iter_per_joint_array(5):sawyer_limits(5,2)
                    % Compute transform T_1_5
                    T_1_5 = T_1_4 * SslStandardDhToTransf(sawyerDH(5,1), sawyerDH(5,2), sawyerDH(5,3), sawyerDH(5,4)+th5);

                    for th6=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                        % Compute transform T_1_6
                        T_1_6 = T_1_5 * SslStandardDhToTransf(sawyerDH(6,1), sawyerDH(6,2), sawyerDH(6,3), sawyerDH(6,4)+th6);

                        for th7=sawyer_limits(7,1):iter_per_joint_array(7):sawyer_limits(7,2)
                            % Compute the final transformation T_1_7
                            % Technically unecessary as this only affects
                            % tool orientation
                            T_1_7 = T_1_6 * SslStandardDhToTransf(sawyerDH(7,1), sawyerDH(7,2), sawyerDH(7,3), sawyerDH(7,4)+th7);
                            T_final = T_1_7 * SslStandardDhToTransf(sawyerDH(8,1), sawyerDH(8,2), sawyerDH(8,3), sawyerDH(8,4));
                            sawyer_ws.full.cartx(count) = T_final(1,4); % Save the value in the x vector
                            sawyer_ws.full.carty(count) = T_final(2,4); % Save the value in the y vector
                            sawyer_ws.full.cartz(count) = T_final(3,4); % Save the value in the z vector
                            count = count+1; % iterate up
                        end
                    end
                end
            end
        end
    end
end

sawyer_mat = [sawyer_ws.full.cartx(:), sawyer_ws.full.carty(:), sawyer_ws.full.cartz(:)];
sawyer_ws.full_unique.cart = unique(sawyer_mat, "rows", "stable");
length(sawyer_ws.full_unique.cart(:,1))

% Now save the vector outputs
file_name = "data/sawyer_ws";
save(file_name, "sawyer_ws", "-v7.3");

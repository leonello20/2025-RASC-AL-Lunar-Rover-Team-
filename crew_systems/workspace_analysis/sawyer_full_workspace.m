% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %

addpath("./kinematics/")

% RECALL THAT DH TABLES START i=0 IS THE BASE LINK, i.e [0,0,0]
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
% 2 |       0           -pi/2           0.1925          th2-pi/2
% 3 |       0           -pi/2           0.4             th3-pi
% 4 |       0           -pi/2          -0.1685          th4-pi
% 5 |       0           -pi/2           0.4             th5-pi
% 6 |       0           -pi/2           0.1363          th6-pi
% 7 |       0           0               0.11            th7
% T |       0           0               0               thT

sawyerDH = [ 
    0.081       -pi/2           0.317            0;
    0           -pi/2           0.1925          -pi/2;
    0           -pi/2           0.4             -pi;
    0           -pi/2          -0.1685          -pi;
    0           -pi/2           0.4             -pi;
    0           -pi/2           0.1363          -pi;
    0            0              0.13375          0;
    0            0              0                0];


sawyerDH_mod = sawyerDH;

sawyer_limits = [-pi    pi;
                 -pi    pi;
                 -pi    pi;
                 -deg2rad(160) deg2rad(160);
                 -pi    pi;
                 -pi    pi;
                 -pi    pi;
                 -pi    pi];

iter_per_joint_array = [deg2rad(45),deg2rad(20),deg2rad(20),deg2rad(20),deg2rad(20),deg2rad(20),deg2rad(90)];


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
fprintf("Array length: " + int2str(count) + "\n")

% Pre-allocate the array sizes
sawyer_ws.full.cartx = zeros(1,count);
sawyer_ws.full.carty = zeros(1,count);
sawyer_ws.full.cartz = zeros(1,count);
sawyer_ws.full.q = zeros(7,count);

% reset counter
count = 1; 
% Begin the long ass iteration (its so over)
for th1=sawyer_limits(1,1):iter_per_joint_array(1):sawyer_limits(1,2)
    fprintf("Started interation, current J1 angle is " )
    th1
    for th2=sawyer_limits(2,1):iter_per_joint_array(2):sawyer_limits(2,2)
        for th3=sawyer_limits(3,1):iter_per_joint_array(3):sawyer_limits(3,2)
            for th4=sawyer_limits(4,1):iter_per_joint_array(4):sawyer_limits(4,2)
                for th5=sawyer_limits(5,1):iter_per_joint_array(5):sawyer_limits(5,2)
                    for th6=sawyer_limits(6,1):iter_per_joint_array(6):sawyer_limits(6,2)
                        for th7=sawyer_limits(7,1):iter_per_joint_array(7):sawyer_limits(7,2)
                            % Iterate the joint positions (theta values)
                            sawyerDH_mod(1,4) = sawyerDH(1,4) + th1;
                            sawyerDH_mod(2,4) = sawyerDH(2,4) + th2;
                            sawyerDH_mod(3,4) = sawyerDH(3,4) + th3;
                            sawyerDH_mod(4,4) = sawyerDH(4,4) + th4;
                            sawyerDH_mod(5,4) = sawyerDH(5,4) + th5;
                            sawyerDH_mod(6,4) = sawyerDH(6,4) + th6;
                            sawyerDH_mod(7,4) = sawyerDH(7,4) + th7;

    
                            % Compute the final output pose
                            T_final = SslStandardDhTableToTransf(0, 8, sawyerDH_mod);
                            sawyer_ws.full.cartx(count) = T_final(1,4); % Save the value in the x vector
                            sawyer_ws.full.carty(count) = T_final(2,4); % Save the value in the y vector
                            sawyer_ws.full.cartz(count) = T_final(3,4); % Save the value in the z vector
                            % Save the joint state that we are currently at
                            sawyer_ws.full.q(1,count) = sawyerDH(1,4) + th1;
                            sawyer_ws.full.q(2,count) = sawyerDH(2,4) + th2;
                            sawyer_ws.full.q(3,count) = sawyerDH(3,4) + th3;
                            sawyer_ws.full.q(4,count) = sawyerDH(4,4) + th4;
                            sawyer_ws.full.q(5,count) = sawyerDH(5,4) + th5;
                            sawyer_ws.full.q(6,count) = sawyerDH(6,4) + th6;
                            sawyer_ws.full.q(7,count) = sawyerDH(7,4) + th7;
                            count = count + 1; % iterate up
                        end
                    end
                end
            end
        end
    end
end

sawyer_mat = [sawyer_ws.full.cartx(:), sawyer_ws.full.carty(:), sawyer_ws.full.cartz(:)];
cart = unique(sawyer_mat, "rows", "stable");
sawyer_ws.full_unique.cartx(:) = cart(:,1);
sawyer_ws.full_unique.carty(:) = cart(:,2);
sawyer_ws.full_unique.cartz(:) = cart(:,3);

% Now save the vector outputs
file_name = "data/sawyer_ws";
save(file_name, "sawyer_ws", "-v7.3");


clear cart sawyer_mat sawyerDH_mod th1 th2 th3 th4 th5 th6 th7 data_length iter_per_joint_array T_final count file_name i 
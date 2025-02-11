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


% CRANE DH table (for reference)
% __|       a(i-1)      alpha(i-1)      d(i)        theta(i)
% 1 |       0           0               0.3             th1
% 2 |       0           -pi/2           0.0             th2
% 3 |       0            pi/2           3.396 + d3      0
% T |      -0.41914      pi/2          -0.133865        0

craneDH = [
    0            0              0.3             0;
    0           -pi/2           0.0             0;
    0            pi/2           3.396           0;
   -0.1         -pi/2           0              -1.261657;
   -0.34         pi/2           0.1             0];

crane_limits = [deg2rad(-30)    deg2rad(30);
                -pi/2            0;
                 0               2.264];

cart_placement = [1, 0, 0.5];
pre_mult = [1 0 0 cart_placement(1);
            0 1 0 cart_placement(2);
            0 0 1 cart_placement(3);
            0 0 0        1         ];


craneDH_mod = craneDH;

iter_per_joint_array = [0.01,0.01,0.01];


% Get the recquired vector length so we don't make it too big
if(length(iter_per_joint_array) ~= 3)
    error("iter_per_joint_array must be length of 3! must contain desired iterations for each joint")
end

count = 1;
for th1=crane_limits(1,1):iter_per_joint_array(1):crane_limits(1,2)
    for th2=crane_limits(2,1):iter_per_joint_array(2):crane_limits(2,2)
        for th3=crane_limits(3,1):iter_per_joint_array(3):crane_limits(3,2)
            count = count + 1;
        end
    end
end

data_length = count; % Get the length of the robit arm
fprintf("Array length: " + int2str(data_length) + "\n")

% initialize our vectors
crane_ws.full.cartx(data_length) = 0; % initialize the crane_ws.full.cartesian x vector
crane_ws.full.carty(data_length) = 0; % initialize the crane_ws.full.cartesian y vector
crane_ws.full.cartz(data_length) = 0; % initialize the crane_ws.full.cartesian z vector

% reset counter
count = 1; 
% Begin the long ass iteration (its so over)
for th1=crane_limits(1,1):iter_per_joint_array(1):crane_limits(1,2)
    fprintf("Started interation, current J1 angle is ")
    th1

    % compute the transform T1
    T1 = SslModifDhToTransf(craneDH(1,1), craneDH(1,2), craneDH(1,3), craneDH(1,4)+th1);

    for th2=crane_limits(2,1):iter_per_joint_array(2):crane_limits(2,2)
        % Compute transform T_1_2
        T_1_2 = T1*SslModifDhToTransf(craneDH(2,1), craneDH(2,2), craneDH(2,3), craneDH(2,4)+th2);

        for d3=crane_limits(3,1):iter_per_joint_array(3):crane_limits(3,2)
            % Compute transform T_1_3
            T_1_3 = T_1_2 * SslModifDhToTransf(craneDH(3,1), craneDH(3,2), craneDH(3,3)+d3, craneDH(3,4));
            
            % Technically unecessary as this only affects
            % tool orientation
            T_final = T_1_3 * SslModifDhTableToTransf(4, 5, craneDH);
            crane_ws.full.cartx(count) = T_final(1,4); % Save the value in the x vector
            crane_ws.full.carty(count) = T_final(2,4); % Save the value in the y vector
            crane_ws.full.cartz(count) = T_final(3,4); % Save the value in the z vector
            count = count+1; % iterate up
        end
    end
end

crane_mat = [crane_ws.full.cartx(:), crane_ws.full.carty(:), crane_ws.full.cartz(:)];
crane_ws.full_unique.cart = unique(crane_mat, "rows", "stable");

% Now save the vector outputs
file_name = "data/crane_ws";
save(file_name, "crane_ws", "-v7.3");

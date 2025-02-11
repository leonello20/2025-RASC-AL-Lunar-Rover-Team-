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

% NBV DH table (for reference)
% __|      a(i-1),            alpha(i-1),       d(i),         theta(i)
% 1 |       0.0,                0.0,            0.2872,        0.0; 
% 2 |       0.0,                1.5708,         0.0,           0.0;  
% 3 |       0.5589,            -3.1415,         0.0,           0.0;    
% 4 |       0.1514,             1.5708,         0.5388,        0.0;  
% 5 |       0.0,               -1.5708,         0.0,           0.0;
% 6 |       0.0,                1.5708,         0.0,           0.0;  
% T |       0.2666               0.0            0.0            0.0;



nbvDH = [
  0.0,        0.0,          0.2872,        0.0; 
  0.0,        pi/2,         0.0,           0.0;  
  0.5589,    -pi,           0.0,           0.0;    
  0.1514,     pi/2,         0.5388,        0.0;  
  0.0,       -pi/2,         0.0,           0.0;
  0.0,        pi/2,         0.0,           0.0;
  0.0,        0.0,          0.2666,        0.0];

nbvDH_mod = nbvDH;

% nbvLimits = [-pi    pi;
%              (0-25)    180+25;
%              -90 90;
%              -180    180;
%               -90-30     90+30;
%              -180    180];

nbv_limits = [-pi    pi;
      -deg2rad(25)   pi+deg2rad(25);
            -pi/2    pi/2;
              -pi    pi;
 -pi/2-deg2rad(25)   pi/2+deg2rad(25);
              -pi    pi];

iter_per_joint_array = [deg2rad(30),deg2rad(15),deg2rad(15),deg2rad(15),deg2rad(15),deg2rad(15)];


% Get the recquired vector length so we don't make it too big
if(length(iter_per_joint_array) ~= 6)
    error("NBV's iter_per_joint_array must be length of 6! must contain desired iterations for each joint")
end

count = 1;
for th1=nbv_limits(1,1):iter_per_joint_array(1):nbv_limits(1,2)
    for th2=nbv_limits(2,1):iter_per_joint_array(2):nbv_limits(2,2)
        for th3=nbv_limits(3,1):iter_per_joint_array(3):nbv_limits(3,2)
            for th4=nbv_limits(4,1):iter_per_joint_array(4):nbv_limits(4,2)
                for th5=nbv_limits(5,1):iter_per_joint_array(5):nbv_limits(5,2)
                    for th6=nbv_limits(6,1):iter_per_joint_array(6):nbv_limits(6,2)
                        count = count+1;
                    end
                end
            end
        end
    end
end

data_length = count; % Get the length of the robit arm
fprintf("Array length: " + int2str(data_length) + "\n")

% initialize our vectors
nbv_ws.full.cartx(data_length) = 0; % initialize the nbv_ws.full.cartesian x vector
nbv_ws.full.carty(data_length) = 0; % initialize the nbv_ws.full.cartesian y vector
nbv_ws.full.cartz(data_length) = 0; % initialize the nbv_ws.full.cartesian z vector

% reset counter
count = 1; 
% Begin the long ass iteration (its so over)
for th1=nbv_limits(1,1):iter_per_joint_array(1):nbv_limits(1,2)
    fprintf("Started interation, current J1 angle is ")
    th1

    % compute the transform T1
    T1 = SslStandardDhToTransf(nbvDH(1,1), nbvDH(1,2), nbvDH(1,3), nbvDH(1,4)+th1);

    for th2=nbv_limits(2,1):iter_per_joint_array(2):nbv_limits(2,2)
        % Compute transform T_1_2
        T_1_2 = T1*SslStandardDhToTransf(nbvDH(2,1), nbvDH(2,2), nbvDH(2,3), nbvDH(2,4)+th2);

        for th3=nbv_limits(3,1):iter_per_joint_array(3):nbv_limits(3,2)
            % Compute transform T_1_3
            T_1_3 = T_1_2 * SslStandardDhToTransf(nbvDH(3,1), nbvDH(3,2), nbvDH(3,3), nbvDH(3,4)+th3);
            
            for th4=nbv_limits(4,1):iter_per_joint_array(4):nbv_limits(4,2)
                % Compute transform T_1_4
                T_1_4 = T_1_3 * SslStandardDhToTransf(nbvDH(4,1), nbvDH(4,2), nbvDH(4,3), nbvDH(4,4)+th4);

                for th5=nbv_limits(5,1):iter_per_joint_array(5):nbv_limits(5,2)
                    % Compute transform T_1_5
                    T_1_5 = T_1_4 * SslStandardDhToTransf(nbvDH(5,1), nbvDH(5,2), nbvDH(5,3), nbvDH(5,4)+th5);

                    for th6=nbv_limits(6,1):iter_per_joint_array(6):nbv_limits(6,2)
                        % Compute transform T_1_6
                        T_1_6 = T_1_5 * SslStandardDhToTransf(nbvDH(6,1), nbvDH(6,2), nbvDH(6,3), nbvDH(6,4)+th6);
                        % Compute the final transformation T_1_7
                        % Technically unecessary as this only affects
                        % tool orientation
                        T_final = T_1_6 * SslStandardDhToTransf(nbvDH(7,1), nbvDH(7,2), nbvDH(7,3), nbvDH(7,4));

                        nbv_ws.full.cartx(count) = T_final(1,4); % Save the value in the x vector
                        nbv_ws.full.carty(count) = T_final(2,4); % Save the value in the y vector
                        nbv_ws.full.cartz(count) = T_final(3,4); % Save the value in the z vector
                        count = count+1; % iterate up
                    end
                end
            end
        end
    end
end

nbv_mat = [nbv_ws.full.cartx(:), nbv_ws.full.carty(:), nbv_ws.full.cartz(:)];
nbv_ws.full_unique.cart = unique(nbv_mat, "rows", "stable");
length(nbv_ws.full_unique.cart(:,1))

% Now save the vector outputs
file_name = "data/nbv_ws";
save(file_name, "nbv_ws", "-v7.3");

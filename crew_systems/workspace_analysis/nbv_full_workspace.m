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

iter_per_joint_array = [deg2rad(12),deg2rad(10),deg2rad(10),deg2rad(12),deg2rad(10),deg2rad(45)];


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
fprintf("Array length: " + int2str(count) + "\n")

% Pre-allocate the array sizes
nbv_ws.full.cartx = zeros(1,count);
nbv_ws.full.carty = zeros(1,count);
nbv_ws.full.cartz = zeros(1,count);
nbv_ws.full.q = zeros(6,count);

% reset counter
count = 1; 
% Begin the long ass iteration (its so over)
for th1=nbv_limits(1,1):iter_per_joint_array(1):nbv_limits(1,2)
    fprintf("Started interation, current J1 angle is ")
    th1
    for th2=nbv_limits(2,1):iter_per_joint_array(2):nbv_limits(2,2)
        for th3=nbv_limits(3,1):iter_per_joint_array(3):nbv_limits(3,2)
            for th4=nbv_limits(4,1):iter_per_joint_array(4):nbv_limits(4,2)
                for th5=nbv_limits(5,1):iter_per_joint_array(5):nbv_limits(5,2)
                    for th6=nbv_limits(6,1):iter_per_joint_array(6):nbv_limits(6,2)
                        % Iterate the joint positions (theta values)
                        nbvDH_mod(1,4) = nbvDH(1,4)+th1;
                        nbvDH_mod(2,4) = nbvDH(2,4)+th2;
                        nbvDH_mod(3,4) = nbvDH(3,4)+th3;
                        nbvDH_mod(4,4) = nbvDH(4,4)+th4;
                        nbvDH_mod(5,4) = nbvDH(5,4)+th5;
                        nbvDH_mod(6,4) = nbvDH(6,4)+th6;

                        % Compute the final output pose
                        T_final = SslModifDhTableToTransf(0, 7, nbvDH_mod);
                        nbv_ws.full.cartx(count) = T_final(1,4); % Save the value in the x vector
                        nbv_ws.full.carty(count) = T_final(2,4); % Save the value in the y vector
                        nbv_ws.full.cartz(count) = T_final(3,4); % Save the value in the z vector
                        % Save the joint state that we are currently at
                        nbv_ws.full.q(1,count) = nbvDH(1,4)+th1;
                        nbv_ws.full.q(2,count) = nbvDH(2,4)+th2;
                        nbv_ws.full.q(3,count) = nbvDH(3,4)+th3;
                        nbv_ws.full.q(4,count) = nbvDH(4,4)+th4;
                        nbv_ws.full.q(5,count) = nbvDH(5,4)+th5;
                        nbv_ws.full.q(6,count) = nbvDH(6,4)+th6;
                        count = count + 1; % iterate up

                    end
                end
            end
        end
    end
end

nbv_mat = [nbv_ws.full.cartx(:), nbv_ws.full.carty(:), nbv_ws.full.cartz(:)];
cart = unique(nbv_mat, "rows", "stable");
nbv_ws.full_unique.cartx(:) = cart(:,1);
nbv_ws.full_unique.carty(:) = cart(:,2);
nbv_ws.full_unique.cartz(:) = cart(:,3);

% Now save the vector outputs
file_name = "data/nbv_ws";
save(file_name, "nbv_ws", "-v7.3");

clear cart nbv_mat nbvDH_mod th1 th2 th3 th4 th5 th6 data_length iter_per_joint_array T_final count file_name i 
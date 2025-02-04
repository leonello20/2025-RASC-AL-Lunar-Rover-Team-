% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %

iter_per_joint_array = [1, 1, 1, 1, 1, 1, 1];


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


% NBV stuff:
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
  0.0,        0.0,            0.2872,        0.0; 
  0.0,        1.5708,         0.0,           0.0;  
  0.5589,    -3.1415,         0.0,           0.0;    
  0.1514,     1.5708,         0.5388,        0.0;  
  0.0,       -1.5708,         0.0,           0.0;
  0.0,        1.5708,         0.0,           0.0;
  0.0,        0.0,            0.2666,        0.0];


nbvLimits = [-180    180;
                0    180;
              -90     90;
             -180    180;
              -90     90;
             -180    180];

% Get the length so we don't make it too big
if(length(iter_per_joint_array) ~= 6)
    error("NBVs iter_per_joint_array must be length of 6! must contain desired iterations for each joint")
end
count = 1;
for th1=nbvLimits(1,1):iter_per_joint_array(1):nbvLimits(1,2)
    for th2=nbvLimits(2,1):iter_per_joint_array(2):nbvLimits(2,2)
        for th3=nbvLimits(3,1):iter_per_joint_array(3):nbvLimits(3,2)
            for th4=nbvLimits(4,1):iter_per_joint_array(4):nbvLimits(4,2)
                for th5=nbvLimits(5,1):iter_per_joint_array(5):nbvLimits(5,2)
                    for th6=nbvLimits(6,1):iter_per_joint_array(6):nbvLimits(6,2)
                        count = count+1;
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
for th1=nbvLimits(1,1):iter_per_joint_array(1):nbvLimits(1,2)
    fprintf("Started interation, current J1 angle is " + int2str(th1) + "\n")
    for th2=nbvLimits(2,1):iter_per_joint_array(2):nbvLimits(2,2)
        for th3=nbvLimits(3,1):iter_per_joint_array(3):nbvLimits(3,2)
            for th4=nbvLimits(4,1):iter_per_joint_array(4):nbvLimits(4,2)
                for th5=nbvLimits(5,1):iter_per_joint_array(5):nbvLimits(5,2)
                    for th6=nbvLimits(6,1):iter_per_joint_array(6):nbvLimits(6,2)
                        % Get the DH transformation matricies
                        T1 = SslRotMat(nbvDH(1,2), "x", "deg")*SslTransMat(nbvDH(1,1), "a")*SslRotMat(th1, "z", "deg")*SslTransMat(nbvDH(1,3), "d");
                        T2 = SslRotMat(nbvDH(2,2), "x", "deg") * SslTransMat(nbvDH(2,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(2,3), "d");
                        T3 = SslRotMat(nbvDH(3,2), "x", "deg") * SslTransMat(nbvDH(3,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(3,3), "d");
                        T4 = SslRotMat(nbvDH(4,2), "x", "deg") * SslTransMat(nbvDH(4,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(4,3), "d");
                        T5 = SslRotMat(nbvDH(5,2), "x", "deg") * SslTransMat(nbvDH(5,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(5,3), "d");
                        T6 = SslRotMat(nbvDH(6,2), "x", "deg") * SslTransMat(nbvDH(6,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(6,3), "d");
                        Tool = SslRotMat(nbvDH(7,2), "x", "deg")*SslTransMat(nbvDH(7,1), "a")*SslRotMat(nbvDH(7,4), "z", "deg")*SslTransMat(nbvDH(7,3), "d");
                        T_final = T1*T2*T3*T4*T5*T6*Tool; % get the final transformation matrix
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

% Now save the vector outputs
file_name = robot_name + "_workspace_data.mat";
nbvx = cartx;
nbvy = carty;
nbvz = cartz;
nbv_cart_coords(:,1) = nbvx;
nbv_cart_coords(:,2) = nbvy;
nbv_cart_coords(:,3) = nbvz;
save(file_name, "nbvx", "nbvy", "nbvz");
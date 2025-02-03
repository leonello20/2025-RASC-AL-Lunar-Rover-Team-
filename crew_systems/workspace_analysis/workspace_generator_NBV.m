% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %

iter_per_joint_array = [1, 1, 1, 1, 1, 1, 1];


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


% NBV stuff:
% NBV DH table (for reference)
% __|       d(i)        theta(i)       a(i-1)      alpha(i-1)
% 1 |     0.2500          th1             0           0
% 2 |       0             th2             0           90
% 3 |       0             th3           0.5589        0
% 4 |     0.5388          th4           0.1514       -90
% 5 |       0             th5             0           90
% 6 |       0             th6             0           90
% T |     0.2666           0              0           0

nbvDH = [ 0.2500     0        0        0  ;
            0        0        0       90  ;
            0        0      0.5589     0  ;
          0.5388     0      0.1514   -90  ;
            0        0        0       90  ;
            0        0        0       90  ;
          0.2666     0        0        0 ];

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
                        T1 = RotMat(nbvDH(1,4), "x", "deg")*TransMat(nbvDH(1,3), "a")*RotMat(th1, "z", "deg")*TransMat(nbvDH(1,1), "d");
                        T2 = RotMat(nbvDH(2,4), "x", "deg")*TransMat(nbvDH(2,3), "a")*RotMat(th2, "z", "deg")*TransMat(nbvDH(2,1), "d");
                        T3 = RotMat(nbvDH(3,4), "x", "deg")*TransMat(nbvDH(3,3), "a")*RotMat(th3, "z", "deg")*TransMat(nbvDH(3,1), "d");
                        T4 = RotMat(nbvDH(4,4), "x", "deg")*TransMat(nbvDH(4,3), "a")*RotMat(th4, "z", "deg")*TransMat(nbvDH(4,1), "d");
                        T5 = RotMat(nbvDH(5,4), "x", "deg")*TransMat(nbvDH(5,3), "a")*RotMat(th5, "z", "deg")*TransMat(nbvDH(5,1), "d");
                        T6 = RotMat(nbvDH(6,4), "x", "deg")*TransMat(nbvDH(6,3), "a")*RotMat(th6, "z", "deg")*TransMat(nbvDH(6,1), "d");
                        Tool = RotMat(nbvDH(7,4), "x", "deg")*TransMat(nbvDH(7,3), "a")*RotMat(nbvDH(7,2), "z", "deg")*TransMat(nbvDH(7,1), "d");
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
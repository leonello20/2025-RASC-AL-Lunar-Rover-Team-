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
                 3.396          3.396+2.264];

cart_placement = [1, 0, 0.5];
pre_mult = [1 0 0 cart_placement(1);
            0 1 0 cart_placement(2);
            0 0 1 cart_placement(3);
            0 0 0        1         ];

% Reset DH params
craneDH_mod = craneDH;
craneDH_mod(3, 3) = craneDH(3, 3) + 2.264; % Set arm to max distance
craneDH_mod(2, 4) = -pi/2; % Set yaw to max reach

count = 1;
% Get shoulder YAW reachable workspace
for theta=crane_limits(1,1):0.001:crane_limits(1,2)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(craneDH(3, 3) + 2.264):-0.001:craneDH(3, 3)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(1,2):-0.001:crane_limits(1,1)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=craneDH(3, 3):0.001:(craneDH(3, 3)  + 2.264)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end


% Reset DH params
craneDH_mod = craneDH;
craneDH_mod(3, 3) = craneDH(3, 3) + 2.264; % Set arm to max distance

count = 1;
% Get shoulder YAW reachable workspace
for theta=crane_limits(2,1):0.001:crane_limits(2,2)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(craneDH(3, 3) + 2.264):-0.001:craneDH(3, 3)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(2,2):-0.001:crane_limits(2,1)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=craneDH(3, 3):0.001:(craneDH(3, 3) + 2.264)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% Now save the vector outputs
file_name = "data/crane_reach";
save(file_name, "crane_reach", "-v7.3");

                 
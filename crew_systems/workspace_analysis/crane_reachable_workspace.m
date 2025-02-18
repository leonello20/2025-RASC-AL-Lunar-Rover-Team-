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
    0            pi/2           3.396           0];

craneDH_mod = craneDH;

max_extension = 2.264;
crane_limits = [deg2rad(-30)    deg2rad(30);
                -pi/2            0;
                 3.396          3.396 + max_extension]; 

% Rover: length = 2.6m, height=0.5, width=1.3 (not super important)
cart_placement = [1, 0, 0.5]; % cartesian offset
pre_mult = [1 0 0 cart_placement(1);
            0 1 0 cart_placement(2);
            0 0 1 cart_placement(3);
            0 0 0        1         ];

% Reset DH params
craneDH_mod(3, 3) = craneDH(3, 3) + max_extension; % Set arm to max distance
craneDH_mod(2, 4) = -pi/2; % Set yaw to max reach

count = 1;
% Get shoulder YAW reachable workspace
for theta=crane_limits(1,1):0.001:crane_limits(1,2)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(crane_limits(3,2)):-0.001:crane_limits(3,1)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(1,2):-0.001:crane_limits(1,1)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=crane_limits(3,1):0.001:crane_limits(3,2)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end


% Reset DH params
craneDH_mod = craneDH;
craneDH_mod(3, 3) = craneDH(3, 3) + 2.264; % Set arm to max distance

count = 1;
% Get shoulder PITCH reachable workspace
for theta=crane_limits(2,1):0.001:crane_limits(2,2)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(craneDH(3, 3) + 2.264):-0.001:craneDH(3, 3)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(2,2):-0.001:crane_limits(2,1)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=craneDH(3, 3):0.001:(craneDH(3, 3) + 2.264)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old1.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old1.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old1.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% Now save the vector outputs
file_name = "data/crane_reach_old1";
save(file_name, "crane_reach_old1", "-v7.3");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% now do the second crane version lol
craneDH = [
    0            0              0.3             0;
    0           -pi/2           0.0             0;
    0            pi/2           1.6             0];

craneDH_mod = craneDH;


% minimum reach: 1.6, maximum reach: 5.28, max extension = 3.68 (meters)
max_extension = 3.68;
crane_limits = [deg2rad(-30)    deg2rad(30);
                -pi/2            0;
                 1.6          1.6 + max_extension]; 

% Rover: length = 2.6m, height=0.5, width=1.3 (not super important)
% yaw joint is 0.2 m, so offset to edge of rover from center is 1.3 - 0.2
cart_placement = [1.1, 0, 0.5]; % cartesian offset
pre_mult = [1 0 0 cart_placement(1);
            0 1 0 cart_placement(2);
            0 0 1 cart_placement(3);
            0 0 0        1         ];

% Reset DH params
craneDH_mod(3, 3) = craneDH(3, 3) + max_extension; % Set arm to max distance
craneDH_mod(2, 4) = -pi/2; % Set yaw to max reach

count = 1;
% Get shoulder YAW reachable workspace
for theta=crane_limits(1,1):0.001:crane_limits(1,2)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(crane_limits(3,2)):-0.001:crane_limits(3,1)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(1,2):-0.001:crane_limits(1,1)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=crane_limits(3,1):0.001:crane_limits(3,2)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end


% Reset DH params
craneDH_mod = craneDH;
craneDH_mod(3, 3) = craneDH(3, 3) + max_extension; % Set arm to max distance

count = 1;
% Get shoulder PITCH reachable workspace
for theta=crane_limits(2,1):0.001:crane_limits(2,2)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(craneDH(3, 3) + 2.264):-0.001:craneDH(3, 3)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(2,2):-0.001:crane_limits(2,1)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=craneDH(3, 3):0.001:(craneDH(3, 3) + 2.264)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

%% add the rigid link
craneDH = [
    0            0              0.3             0;
    0           -pi/2           0.0             0;
    0            pi/2           1.6             0;
   -0.9317      -pi/2           0.0             0;
    0.0          pi/2          -0.3653          0];

craneDH_mod = craneDH;


% minimum reach: 1.6, maximum reach: 5.28, max extension = 3.68 (meters)
max_extension = 3.68;
crane_limits = [deg2rad(-30)    deg2rad(30);
                -pi/2            0;
                 1.6          1.6 + max_extension]; 

% Rover: length = 2.6m, height=0.5, width=1.3 (not super important)
% yaw joint is 0.2 m, so offset to edge of rover from center is 1.3 - 0.2
cart_placement = [1.1, 0, 0.5]; % cartesian offset
pre_mult = [1 0 0 cart_placement(1);
            0 1 0 cart_placement(2);
            0 0 1 cart_placement(3);
            0 0 0        1         ];

% Reset DH params
craneDH_mod(3, 3) = craneDH(3, 3) + max_extension; % Set arm to max distance
craneDH_mod(2, 4) = -pi/2; % Set yaw to max reach

count = 1;
% Get shoulder YAW reachable workspace
for theta=crane_limits(1,1):0.001:crane_limits(1,2)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.yaw_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(crane_limits(3,2)):-0.001:crane_limits(3,1)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.yaw_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(1,2):-0.001:crane_limits(1,1)
    % Get the DH transformation matricies
    craneDH_mod(1, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.yaw_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=crane_limits(3,1):0.001:crane_limits(3,2)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.yaw_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.yaw_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.yaw_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end


% Reset DH params
craneDH_mod = craneDH;
craneDH_mod(3, 3) = craneDH(3, 3) + max_extension; % Set arm to max distance

count = 1;
% Get shoulder PITCH reachable workspace
for theta=crane_limits(2,1):0.001:crane_limits(2,2)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.pitch_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=(craneDH(3, 3) + 2.264):-0.001:craneDH(3, 3)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 3, craneDH_mod);

    crane_reach_old2.pitch_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for theta=crane_limits(2,2):-0.001:crane_limits(2,1)
    % Get the DH transformation matricies
    craneDH_mod(2, 4) = theta;
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.pitch_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
for d=craneDH(3, 3):0.001:(craneDH(3, 3) + 2.264)
    % Get the DH transformation matricies
    craneDH_mod(3, 3) = d; % Set arm to max distance
    T_final = pre_mult*SslModifDhTableToTransf(0, 5, craneDH_mod);

    crane_reach_old2.pitch_rigid.cartx(count) = T_final(1,4); % Save the value in the x vector
    crane_reach_old2.pitch_rigid.carty(count) = T_final(2,4); % Save the value in the y vector
    crane_reach_old2.pitch_rigid.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% Now save the vector outputs
file_name = "data/crane_reach_old2";
save(file_name, "crane_reach_old2", "-v7.3");

                 
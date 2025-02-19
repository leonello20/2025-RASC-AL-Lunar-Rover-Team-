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

% DYMAFLIGHT DH table (for reference)
% __|       a(i-1)      alpha(i-1)      d(i)        theta(i)
% 1 |       0           0               0.18161        th1
% 2 |       0           pi/2            0.0            th2
% 3 |       0          -pi/2            0.430022       th3
% 4 |      -0.041402    pi/2            0.0            th4
% 5 |       0.041402   -pi/2            0.37084        th5
% 6 |       0.033274    pi/2            0.0            th6
% 7 |      -0.033274   -pi/2            0.0            th7
% T |       0           0               0.314325       0

dymaDH = [
    0            0              0.18161         0;
    0            pi/2           0.0             0;
    0           -pi/2           0.430022        0;
   -0.041402     pi/2           0.0             0;
    0.041402    -pi/2           0.37084         0;
    0.033274     pi/2           0.0             0;
   -0.033274    -pi/2           0.0             0;
    0            0              0.314325        0];

dymaDH_mod = dymaDH;

count = 1;
% Get shoulder YAW reachable workspace
dymaDH_mod(2, 4) = pi/2;
for theta=-pi:0.001:pi
    % Get the DH transformation matricies
    dymaDH_mod(1, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 8, dymaDH_mod);

    dyma_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    dyma_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    dyma_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% reset DH table
dymaDH_mod = dymaDH;

count = 1;
for theta=-deg2rad(120):0.001:deg2rad(120)

    % Get the DH transformation matricies
    dymaDH_mod(2, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 8, dymaDH_mod);

    dyma_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    dyma_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    dyma_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% reset DH table
dymaDH_mod = dymaDH;
dymaDH_mod(2, 4) = deg2rad(120);
count = 1;
for theta=0:0.001:deg2rad(155)
    % Get the DH transformation matricies
    dymaDH_mod(4, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 8, dymaDH_mod);

    dyma_reach.elbow.cartx(count) = T_final(1,4); % Save the value in the x vector
    dyma_reach.elbow.carty(count) = T_final(2,4); % Save the value in the y vector
    dyma_reach.elbow.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end

% reset DH table
dymaDH_mod = dymaDH;
dymaDH_mod(2, 4) = deg2rad(120);
dymaDH_mod(4, 4) = deg2rad(155);
dymaDH_mod(5, 4) = pi;
count = 1;
for theta=-deg2rad(65):0.001:0
    % Get the DH transformation matricies
    dymaDH_mod(6, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 8, dymaDH_mod);

    dyma_reach.wrist.cartx(count) = T_final(1,4); % Save the value in the x vector
    dyma_reach.wrist.carty(count) = T_final(2,4); % Save the value in the y vector
    dyma_reach.wrist.cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end


% Now save the vector outputs
file_name = "data/dyma_reach";
save(file_name, "dyma_reach", "-v7.3");

clear dymaDH_mod count theta file_name
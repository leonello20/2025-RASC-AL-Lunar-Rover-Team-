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
  0.0,        0.0,          0.2872,        0.0; 
  0.0,        pi/2,         0.0,           0.0;  
  0.5589,    -pi,           0.0,           0.0;    
  0.1514,     pi/2,         0.5388,        0.0;  
  0.0,       -pi/2,         0.0,           0.0;
  0.0,        pi/2,         0.0,           0.0;
  0.0,        0.0,          0.2666,        0.0];



% nbvLimits = [-pi    pi;
%              (0-25)    180+25;
%              -90 90;
%              -180    180;
%               -90-30     90+30;
%              -180    180];

% Begin the long ass iteration (its so over)

nbvDH_mod = nbvDH;
nbvDH_mod(2, 4) = pi;
nbvDH_mod(3, 4) = pi/2;

count = 1;
% Get shoulder YAW reachable workspace
for theta=-pi:0.001:pi

    % Get the DH transformation matricies

    nbvDH_mod(1, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 7, nbvDH_mod);
    
    nbv_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
    nbv_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
    nbv_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
    nbv_reach.yaw.q(count, :) = [nbvDH_mod(1,4),nbvDH_mod(2,4),nbvDH_mod(3,4),nbvDH_mod(4,4),nbvDH_mod(5,4),nbvDH_mod(6,4)];
    count = count+1; % iterate up
end


% reset DH table
nbvDH_mod = nbvDH;
nbvDH_mod(3, 4) = pi/2;

count = 1;
for theta=(-deg2rad(25)):0.001:(pi+deg2rad(25))

    % Get the DH transformation matricies
    nbvDH_mod(2, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 7, nbvDH_mod);
    
    nbv_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
    nbv_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
    nbv_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
    nbv_reach.pitch.q(count, :) = [nbvDH_mod(1,4),nbvDH_mod(2,4),nbvDH_mod(3,4),nbvDH_mod(4,4),nbvDH_mod(5,4),nbvDH_mod(6,4)];
    count = count+1; % iterate up
end


% reset DH table
nbvDH_mod = nbvDH;
nbvDH_mod(2, 4) = pi+deg2rad(25);

count = 1;
for theta=(-pi/2):0.001:pi/2

    % Get the DH transformation matricies
    nbvDH_mod(3, 4) = theta;
    T_final = SslModifDhTableToTransf(0, 7, nbvDH_mod);
    
    nbv_reach.elbow.cartx(count) = T_final(1,4); % Save the value in the x vector
    nbv_reach.elbow.carty(count) = T_final(2,4); % Save the value in the y vector
    nbv_reach.elbow.cartz(count) = T_final(3,4); % Save the value in the z vector
    nbv_reach.elbow.q(count, :) = [nbvDH_mod(1,4),nbvDH_mod(2,4),nbvDH_mod(3,4),nbvDH_mod(4,4),nbvDH_mod(5,4),nbvDH_mod(6,4)];
    count = count+1; % iterate up
end

% Now save the vector outputs
file_name = "data/nbv_reach";
save(file_name, "nbv_reach", "-v7.3");
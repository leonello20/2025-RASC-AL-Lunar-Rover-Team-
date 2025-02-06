% --------------------- %
% ---- ENAE484 TEAM --- %
%  Workspace generator  %
% --------------------- %
% --------------------- %
clear

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
% 2 |       0           -pi/2           0.1925          th2
% 3 |       0           -pi/2           0.4             th3
% 4 |       0           -pi/2          -0.1685          th4
% 5 |       0           -pi/2           0.4             th5
% 6 |       0           -pi/2           0.1363          th6
% 7 |       0           -pi/2           0.11            th7
% T |       0           0               0               thT



sawyerDH = [ 
    0.081       -pi/2           0.317           0;
    0           -pi/2           0.1925          3*pi/2;
    0           -pi/2           0.4             0;
    0           -pi/2          -0.1685          pi;
    0           -pi/2           0.4             0;
    0           -pi/2           0.1363          pi;
    0           0               0.13375         3*pi/2;
    0           0               0               0];

% % 0.4 
% sawyerDH = [ 
%     0           0               0.317           0;
%     0.081       -pi/2           0.1925          0;
%     0           0           0               0;
%     0           -pi/2          -0.1685          0;
%     0           -pi/2           0.4             0;
%     0           -pi/2           0.1363          0;
%     0           0               0.13375         0;
%     0           0               0               0];


% Begin the long ass iteration (its so over)

% sawyerDH_mod = sawyerDH;
% sawyerDH_mod(2, 4) = pi;
% sawyerDH_mod(3, 4) = pi/2;
% 
% count = 1;
% % Get shoulder YAW reachable workspace
% for theta=-pi:0.001:pi
% 
%     % Get the DH transformation matricies
% 
%     sawyerDH_mod(1, 4) = theta;
%     T_final = SslDhTableToTransf(0, 7, sawyerDH_mod);
% 
%     sawyer_reach.yaw.cartx(count) = T_final(1,4); % Save the value in the x vector
%     sawyer_reach.yaw.carty(count) = T_final(2,4); % Save the value in the y vector
%     sawyer_reach.yaw.cartz(count) = T_final(3,4); % Save the value in the z vector
%     count = count+1; % iterate up
% end
% 
% 
% % reset DH table
% sawyerDH_mod = sawyerDH;
% sawyerDH_mod(3, 4) = pi/2;
% 
% count = 1;
% for theta=(-deg2rad(25)):0.001:(pi+deg2rad(25))
% 
%     % Get the DH transformation matricies
%     sawyerDH_mod(2, 4) = theta;
%     T_final = SslDhTableToTransf(0, 7, sawyerDH_mod);
% 
%     sawyer_reach.pitch.cartx(count) = T_final(1,4); % Save the value in the x vector
%     sawyer_reach.pitch.carty(count) = T_final(2,4); % Save the value in the y vector
%     sawyer_reach.pitch.cartz(count) = T_final(3,4); % Save the value in the z vector
%     count = count+1; % iterate up
% end
% 
% 
% % reset DH table
% sawyerDH_mod = sawyerDH;
% sawyerDH_mod(2, 4) = pi+deg2rad(25);
% 
% count = 1;
% for theta=(-pi/2):0.001:pi/2
% 
%     % Get the DH transformation matricies
%     sawyerDH_mod(3, 4) = theta;
%     T_final = SslDhTableToTransf(0, 7, sawyerDH_mod);
% 
%     sawyer_reach.elbow.cartx(count) = T_final(1,4); % Save the value in the x vector
%     sawyer_reach.elbow.carty(count) = T_final(2,4); % Save the value in the y vector
%     sawyer_reach.elbow.cartz(count) = T_final(3,4); % Save the value in the z vector
%     count = count+1; % iterate up
% end




% Now save the vector outputs
% file_name = "sawyer_yaw_workspace.mat";
% x = cartx;
% y = carty;
% z = cartz;
% save(file_name, "x", "y", "z");
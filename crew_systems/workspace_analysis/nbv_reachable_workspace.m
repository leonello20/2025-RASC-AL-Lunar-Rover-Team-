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


% nbvLimits = [-180    180;
%              (0-30)    180+30;
%              -90 180;
%              -180    180;
%               -90-30     90+30;
%              -180    180];


yaw_plane_cartx = 0;
yaw_plane_carty = 0;
yaw_plane_cartz = 0;
% Begin the long ass iteration (its so over)

T2 = SslRotMat(nbvDH(2,2), "x", "deg") * SslTransMat(nbvDH(2,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(2,3), "d");
T3 = SslRotMat(nbvDH(3,2), "x", "deg") * SslTransMat(nbvDH(3,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(3,3), "d");
T4 = SslRotMat(nbvDH(4,2), "x", "deg") * SslTransMat(nbvDH(4,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(4,3), "d");
T5 = SslRotMat(nbvDH(5,2), "x", "deg") * SslTransMat(nbvDH(5,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(5,3), "d");
T6 = SslRotMat(nbvDH(6,2), "x", "deg") * SslTransMat(nbvDH(6,1), "a") * SslRotMat(0, "z", "rad") * SslTransMat(nbvDH(6,3), "d");
% Tool = SslRotMat(nbvDH(7,2), "x", "deg")*SslTransMat(nbvDH(7,1), "a")*SslRotMat(nbvDH(7,4), "z", "deg")*SslTransMat(nbvDH(7,3), "d");

count = 1;
for th1=-180:0.005:180
    % Get the DH transformation matricies
    T1 = SslRotMat(nbvDH(1,2), "x", "deg")*SslTransMat(nbvDH(1,1), "a")*SslRotMat(th1, "z", "deg")*SslTransMat(nbvDH(1,3), "d");
    T_final = T1*T2*T3*T4*T5*T6;   %*Tool; % get the final transformation matrix
    yaw_plane_cartx(count) = T_final(1,4); % Save the value in the x vector
    yaw_plane_carty(count) = T_final(2,4); % Save the value in the y vector
    yaw_plane_cartz(count) = T_final(3,4); % Save the value in the z vector
    count = count+1; % iterate up
end
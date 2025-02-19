close all
dyma = importDyma();
show(dyma, Frames="off");

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

% ---------------------- %
% --- CHARLIE HANNER --- %
%  DH to Transf Matrix   %
% ---------------------- %
% ---------------------- %

% function to generate the transformation from frame i to frame j, given a
% DH Table (modified DH), IN RADIANS
function mat = SslModifDhTableToTransf(from, to, DH)

% expected input shape:
% __|      a(i-1),            alpha(i-1),       d(i),         theta(i)
% 1 |       a0,                 alpha0,         d1,           theta1 
% 2 |       a1,                 alpha1,         d2,           theta2;  
%   |       .                     .             .               .   
%   |       .                     .             .               .   
%   |       .                     .             .               .  
% j |       aj-1,               alphaj-1,       dj,           thetaj;

% CAN ONLY GO FROM LOWER -> HIGHER FRAMES

T_track = eye(4);

for i = from:to
    if(i == 0)
        alpha = 0;
        a = 0;
        d = 0;
        theta = 0;
    else
        alpha = DH(i,2);
        a = DH(i,1);
        d = DH(i,3);
        theta = DH(i,4);
    end
    T_track = T_track*[cos(theta), -sin(theta), 0, a;
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
     sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
     0, 0, 0, 1];
 
end
mat = T_track;
end
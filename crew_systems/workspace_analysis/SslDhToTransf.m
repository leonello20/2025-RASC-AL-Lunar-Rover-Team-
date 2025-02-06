% ---------------------- %
% --- CHARLIE HANNER --- %
%  DH to Transf Matrix   %
% ---------------------- %
% ---------------------- %

% Function for creating the T matrix from modified DH parameters, in
% RADIANS
function mat = SslDhToTransf(a, alpha, d, theta)    
mat = [cos(theta), -sin(theta), 0, a;
     sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
     sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
     0, 0, 0, 1];
end
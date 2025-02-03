% ------------------------- %
% ---- ROMEO PERLSTEIN ---- %
% Translation Matrix Maker  %
%   UMD Space Systems Lab   %
% ------------------------- %

% Returns a translation matrix
function T = TransMat(DH_value, DH_d_or_a)
    if(DH_d_or_a == "d")
        T = [1 0 0 0; 0 1 0 0; 0 0 1 DH_value; 0 0 0 1];
    elseif(DH_d_or_a == "a")
        T = [1 0 0 DH_value; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    else
        error("[ERROR - TransRot] need to specify if this is the d, or a translation matrix by inputting 'd', or 'a' for DH_d_or_a!")
    end
end
function [R,mH] = Q6(om,C)

R = 1/(om*C);
in = input('Low-pass (L) or High-pass (H)? ','s');

switch in
    case 'L'
        mH = 1/sqrt(1+(om*C*R)^2);
    case 'H'
        mH = om*C*R/sqrt(1+(om*C*R)^2);
    otherwise
        disp('Read the istructions!');
end

end
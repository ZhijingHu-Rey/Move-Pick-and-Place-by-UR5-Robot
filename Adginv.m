function [A] = Adginv(H)
    
    % Calculation of inverse adjoint matrix
    % Information extraction
    R = H(1:3,1:3);
    p = H(1:3,4);
    
    % Calculation and formation of final results
    
    A(1:3,1:3) = R';
    A(4:6,1:3) = zeros(3,3);
    A(4:6,4:6) = R';
    A(1:3,4:6) = -SKEW3(R'*p)*R';

end


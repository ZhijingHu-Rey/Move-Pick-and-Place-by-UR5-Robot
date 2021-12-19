function [Y]=SKEW3(A)

    % Calculation of skew symmetric matrix from rotation matrix
    [rows, cols] = size(A);

    if ((rows ~= 3) | (cols ~= 1))
        error('SKEW3 requires a 3x1 vector argument. Check your dimensions.');
    end
    
    % Returns the vectorized form
    Y = [0 -A(3) A(2);
         A(3) 0 -A(1);
         -A(2) A(1) 0];
end
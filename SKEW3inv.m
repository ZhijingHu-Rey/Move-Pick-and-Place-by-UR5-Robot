function [w] = SKEW3inv(hat)

    % Calculation of rotation axis from skew symmetric
    w = zeros(3,1);

    w(1)=hat(3,2);
    w(2)=hat(1,3);
    w(3)=hat(2,1);
    
end


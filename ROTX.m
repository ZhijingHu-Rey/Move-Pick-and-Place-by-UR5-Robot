function [A] = ROTX(theta)

    A = [1 0 0;
         0 cos(theta) -sin(theta);
         0 sin(theta) cos(theta)];

end
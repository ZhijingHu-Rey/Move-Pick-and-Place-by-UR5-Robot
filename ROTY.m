function [A] = ROTY(s)
    
    A = [cos(s) 0 sin(s);
         0 1 0;
         -sin(s) 0 cos(s)];

end
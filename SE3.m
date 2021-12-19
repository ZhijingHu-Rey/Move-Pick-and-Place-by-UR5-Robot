function [A]=SE3(R,p)
    [rows, cols] = size(R);
    if ((rows ~= 3) | (cols ~= 3))
      error('SE3 requires a 3x3 matrix argument. Check your dimensions.');
    end
    d = det(R);
    if ((d <0.999) | (d >1.001))
      fprintf(1,'Error in SE3: the rotational part is not a rotation matrix. Determinent is %f, should be 1.0\n',d);
      error('aborting');
    end
    [rows, cols] = size(p);
    if ((rows ~= 3) | (cols ~= 1))
      error('SE3 requires a 3x1 vector argument. Check your dimensions.');
    end
    A(1:3,1:3)=R;
    A(1:3,4)=p;
    A(4,1:3)=0;
    A(4,4)=1;
end
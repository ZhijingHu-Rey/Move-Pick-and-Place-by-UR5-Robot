function [xi] = getXi(gst)
    
    % Makes sure that g is a 4x4 matrix
    if norm(size(gst) - [4, 4]) ~= 0
        disp("Invalid input! Check if g is a 4x4 homogeneous transformation matrix")
        return 
    
    % Makes sure that g is a 4x4 homogeneous matrix
    elseif norm(gst(4, 1:4) - [0, 0, 0, 1]) > 1e-8
        disp("Invalid input! Check if g is a homogeneous transformation matrix")
        return
        
    % Makes sure that the top left 3x3 block of g is a rotation matrix
    elseif det(gst(1:3, 1:3)) - 1 > 1e-8 || norm(gst(1:3, 1:3).' * gst(1:3, 1:3) - eye(3)) > 1e-8
        disp("Invalid input! Check if top left 3x3 block is a rotation matrix")
        return
    end
    
    % Parameter initialization
    xi=zeros(6,1);
    
    % Extracts the components of the matrix
    R=gst(1:3,1:3);
    p=gst(1:3,4);
    
    % Computes theta
    theta=acos((trace(R)-1)/2);
    
    % When theta = 0
    if(abs(theta) <= 1e-8)
        xi(1:3) = p;
        return;
    
    % When theta = pi
    elseif (abs(theta - pi) < 1e-10)
        B = 0.5 * (R + eye(3));
        w = sqrt(diag(B));
        if B(3, 2) < 0; w(1) = -w(1); end
        if B(3, 1) < 0; w(2) = -w(2); end
        if B(2, 1) < 0; w(3) = -w(3); end
        A = (eye(3) - R) * SKEW3(w) + w * w.' * theta;
        v = A \ p;
        xi = theta * [v; w];
    
    % Compute omega, and correspondingly v
    else
        w = 1/(2*sin(theta))*SKEW3inv(R-R.');
        v = ((eye(3)-R)*SKEW3(w)+w*w.'*theta)\p;
        xi(1:3) = v;
        xi(4:6) = w;
        xi = xi*theta;
    end
    
    
end


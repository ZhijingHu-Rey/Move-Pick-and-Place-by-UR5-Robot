function[collision]=check_collision(q)
    collision=0;
    
    if((q(3)<-10*pi/11||q(3)>10*pi/11))
        collision=collision+1;
        disp("collision type:2")
        return
    elseif((q(3)<-5*pi/6||q(3)>5*pi/6)&&abs(mod(q(4),pi))>pi/18)
        collision=collision+1;
        disp("collision type:3")
        return
    elseif(abs(q(4))>pi*3/4&&abs(q(5))>pi/2)
        collision=1;
        disp("collision type:4")
        return
    elseif(abs(q(4))>pi*5/6&&abs(q(5))>pi/18)
        collision=1;
        disp("collision type:5")
        return
    end
    [g]=get_gs(q);
    for i=0:5
        if(g(3+i*4,4)<0)
           collision=collision+1;
           disp("collision type:6")
           break;
        end
    end
    return;
end


function [g] = get_gs(theta)
 
    % UR5 link parameters
    l(1) = 0.425;      % L1 link length
    l(2) = 0.392;      % L2 link length
    l(3) = 0.1093;     % L3 link length
    l(4) = 0.09475;    % L4 link length
    l(5) = 0.0825;     % L5 link length
    l0=0.0892;
    % Home configuration gst(0)
    p0=[0;l(3)+l(5);l0+l(1)+l(2)+l(4)];
    gst0 = [expm(SKEW3([1 0 0]')*-pi/2),p0;
            0 0 0 1];
    
    % w = Omega vectors for rotation axes of each revolute joint
    % q = Point on rotation axes of each revolute joint
    w(:,1) = [0; 0; 1]; q(:,1) = [0; 0; 0];
    w(:,2) = [0; 1; 0]; q(:,2) = [0; 0; l0];
    w(:,3) = [0; 1; 0]; q(:,3) = [0; 0; l0+l(1)];
    w(:,4) = [0; 1; 0]; q(:,4) = [0; 0; l0+l(1) + l(2)];
    w(:,5) = [0; 0; 1]; q(:,5) = [0; l(3); l0];
    w(:,6) = [0; 1; 0]; q(:,6) = [0; 0; l0+l(1) + l(2) + l(4)];
    
    xi(1:3,1)=cross(-w(:,1),q(:,1));xi(4:6,1)=w(:,1);
    xi(1:3,2)=cross(-w(:,2),q(:,2));xi(4:6,2)=w(:,2);
    xi(1:3,3)=cross(-w(:,3),q(:,3));xi(4:6,3)=w(:,3);
    xi(1:3,4)=cross(-w(:,4),q(:,4));xi(4:6,4)=w(:,4);
    xi(1:3,5)=cross(-w(:,5),q(:,5));xi(4:6,5)=w(:,5);
    xi(1:3,6)=cross(-w(:,6),q(:,6));xi(4:6,6)=w(:,6);
    g=zeros(24,4);
    % Computes end effector pose gst using the POE formulation by multiplying successive xi_hat matrices together and home configuration g0
    g(21:24,:)=expm(Xihat(xi(:,1))*theta(1))*expm(Xihat(xi(:,2))*theta(2))*expm(Xihat(xi(:,3))*theta(3))*expm(Xihat(xi(:,4))*theta(4))*expm(Xihat(xi(:,5))*theta(5))*expm(Xihat(xi(:,6))*theta(6))*gst0;
    g(17:20,:)=expm(Xihat(xi(:,1))*theta(1))*expm(Xihat(xi(:,2))*theta(2))*expm(Xihat(xi(:,3))*theta(3))*expm(Xihat(xi(:,4))*theta(4))*expm(Xihat(xi(:,5))*theta(5))*gst0;
    g(13:16,:)=expm(Xihat(xi(:,1))*theta(1))*expm(Xihat(xi(:,2))*theta(2))*expm(Xihat(xi(:,3))*theta(3))*expm(Xihat(xi(:,4))*theta(4))*gst0;
    g(9:12,:)=expm(Xihat(xi(:,1))*theta(1))*expm(Xihat(xi(:,2))*theta(2))*expm(Xihat(xi(:,3))*theta(3))*gst0;
    g(5:8,:)=expm(Xihat(xi(:,1))*theta(1))*expm(Xihat(xi(:,2))*theta(2))*gst0;
    g(1:4,:)=expm(Xihat(xi(:,1))*theta(1))*gst0;
end
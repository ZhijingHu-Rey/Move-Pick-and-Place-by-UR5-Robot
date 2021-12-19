% Choose the best joint space
function[bestq,found]=find_bestQ(q)
    values=zeros(1,8);
    found=0;
    % check collisions for eight results
    for i=1:8
        if(check_collision(q(:,i))==0)
            values(i)=norm(q(:,i)-[0;-pi/2;0;0;0;0])+0.1*(q(2,i)+pi/2);
        end
    end
    values(values==0)=inf;
    % choose the minimum joint angles
    [qmin,bestq_index]=min(values);
    if(qmin==inf) 
        found=-1;
        bestq=zeros(6,1);
        return
    end
    bestq=q(:,bestq_index);

end
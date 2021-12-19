function [mu] = manipulability(J,measure)

    % If the specified metric is sigmamin
    if(measure == "sigmamin")
        mu = svds(J,1,"smallest");
    
    % If the specified metric is invcond    
    elseif(measure == "invcond")
        mu = svds(J,1,"smallest") / svds(J,1);
    
    % If the specified metric is detjac
    elseif(measure == "detjac")
        mu=det(J);
    
    % If no argument is given or incorrect argument given
    else
        error("Unsupported measure");
    
    end
end


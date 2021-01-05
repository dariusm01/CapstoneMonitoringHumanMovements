function NewPoints = secondOrderUKFPropagation(dt, sigmaPoints, wk)

F = [1 0 0 dt 0 0 0.5*dt^2 0 0; 
     0 1 0 0 dt 0 0 0.5*dt^2 0; 
     0 0 1 0 0 dt 0 0 0.5*dt^2; 
     0 0 0 1 0 0 dt 0 0; 
     0 0 0 0 1 0 0 dt 0; 
     0 0 0 0 0 1 0 0 dt;
     0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 1];
 
 
propagatedPoints = zeros(size(sigmaPoints));

j = size(sigmaPoints);

    for i = 1:j(2)
        propagatedPoints(:,i) = F*sigmaPoints(:,i);
    end 
    
propagatedPoints = propagatedPoints + wk;
    
NewPoints = propagatedPoints;
end 
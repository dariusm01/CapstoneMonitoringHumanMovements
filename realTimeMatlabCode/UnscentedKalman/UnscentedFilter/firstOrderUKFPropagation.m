function NewPoints = firstOrderUKFPropagation(sigmaPoints, dt, Gyro, wk)

F = eye(3);

G = F*dt;
 
possibleStates = zeros(size(sigmaPoints));

possibleDerivatives = possibleStates;

propagatedPoints = possibleStates;

j = size(sigmaPoints);

% x = Fx + Gu + w

    for i = 1:j(2)
        possibleStates(:,i) = F*sigmaPoints(:,i);
        
        [phiDot,thetaDot,psiDot] = EulerRate(sigmaPoints(1,i),sigmaPoints(2,i), Gyro);
        
        possibleDerivatives(:,i) = G*[phiDot;thetaDot;psiDot];
        
        propagatedPoints(:,i) = possibleStates(:,i) + possibleDerivatives(:,i);
    end 

propagatedPoints = propagatedPoints + wk;
    
NewPoints = propagatedPoints;

end 
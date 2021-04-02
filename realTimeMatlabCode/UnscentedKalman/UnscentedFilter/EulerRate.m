function [phiDot,thetaDot,psiDot] = EulerRate(phi,theta, Gyro)


EulerKinematic = [1 sin(phi)*tan(theta) cos(phi)*tan(theta); 
                  0 cos(phi) -sin(phi);
                  0 sin(phi)*sec(theta) cos(phi)*sec(theta)];
              
EulerRates = EulerKinematic*Gyro;

phiDot = EulerRates(1);
thetaDot = EulerRates(2);
psiDot = EulerRates(3);

end 
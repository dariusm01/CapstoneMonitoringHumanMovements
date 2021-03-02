function [ax,ay,az] = AccelModel(Phi, Theta, Psi)

ax = cos(Theta)*cos(Psi) + cos(Theta)*sin(Psi) - sin(Theta);

ay = (cos(Psi)*sin(Theta)*sin(Phi)) + (cos(Phi)*cos(Phi) + sin(Theta)*sin(Phi)*sin(Psi)) +...
    cos(Theta)*sin(Phi);

az = (cos(Phi)*cos(Psi)*sin(Theta)) - (cos(Psi)*sin(Phi) + cos(Phi)*sin(Theta)*sin(Psi)) +...
    cos(Theta)*cos(Phi);
end 
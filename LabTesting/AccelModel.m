function [ax,ay,az] = AccelModel(Phi, Theta)

ax = - sin(Theta);

ay = cos(Theta)*sin(Phi);

az = cos(Theta)*cos(Phi);
end 
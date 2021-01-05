function angle = accelAngleY(accelvals)

x = accelvals(1);
y = accelvals(2);
z = accelvals(3);

a = sqrt(y^2 + z^2);

b = atan2(-x,a);

angle = rad2deg(b);

end 
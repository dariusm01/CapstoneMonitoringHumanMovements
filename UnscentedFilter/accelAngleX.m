function angle = accelAngleX(accelvals)

x = accelvals(1);
y = accelvals(2);
z = accelvals(3);

a = sqrt(x^2 + z^2);

b = atan2(y,a);

angle = rad2deg(b);

end 
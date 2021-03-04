function H = MeasurementJacobian(phi,theta)

H = [0 -cos(theta) 0; cos(theta)*cos(phi) -sin(theta)*sin(phi) 0; -cos(theta)*sin(phi) -sin(theta)*cos(phi) 0];

end 
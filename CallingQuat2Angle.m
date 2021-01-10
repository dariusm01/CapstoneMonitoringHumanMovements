function EulerAngles = CallingQuat2Angle(q)

[psi, theta, phi] = quat2angle(q);

phi = rad2deg(phi);
theta = rad2deg(theta);
psi = rad2deg(psi);

EulerAngles = [phi;theta;psi];
end 
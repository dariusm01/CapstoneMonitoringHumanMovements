function [w,x,y,z]=defQuaternion(roll, pitch, yaw)

w = cosd(pitch/2)*cosd(yaw/2)*cosd(roll/2)+sind(pitch/2)*sind(yaw/2)*sind(roll/2);
x = sind(pitch/2)*cosd(yaw/2)*cosd(roll/2)-cosd(pitch/2)*sind(yaw/2)*sind(roll/2);
y = cosd(pitch/2)*sind(yaw/2)*cosd(roll/2)+sind(pitch/2)*cosd(yaw/2)*sind(roll/2);
z = cosd(pitch/2)*cosd(yaw/2)*sind(roll/2)-sind(pitch/2)*sind(yaw/2)*cosd(roll/2);
end 
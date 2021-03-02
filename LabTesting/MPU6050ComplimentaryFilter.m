
%% Getting Accelerometer Data (in NED frame)
mat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/z_AxisRotation.xlsx");

AccelX = mat.AccelY;
AccelY = mat.AccelX;
AccelZ = mat.AccelZ * -1;

%% Getting Euler Angles
otherMat = readtable("/Users/dariusmensah/Desktop/LabTesting/MPU_6050/NED_Frame/z_AxisEulerAngles.xlsx");

phi = otherMat.Phi;
theta = otherMat.Theta;

phi_dot = otherMat.PhiDot;
theta_dot = otherMat.ThetaDot;


%% Initial Values 
Phi = phi(1);
Theta = theta(1);

%% Values we want to plot 

PhiAngleComplimentary = [];
ThetaAngleComplimentary  = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.98;

dt = 1/100;

time = 1:length(AccelX);

time = time.'*dt;

%% Complimentary Filter

for i = 1:length(time)

    % angle corrections using accelerometer 
    PhiAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))); 

    ThetaAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2)));  
    
    newAngleX = alpha*(phi_dot(i)*dt+Phi) + (1-alpha)*PhiAccel;
    
    newAngleY = alpha*(theta_dot(i)*dt+Theta) + (1-alpha)*ThetaAccel;
    
    % Store for plotting
    PhiAngleComplimentary = [PhiAngleComplimentary; newAngleX];
    ThetaAngleComplimentary  = [ThetaAngleComplimentary ;newAngleY];
    
    AccelAngleX = [AccelAngleX; PhiAccel];
    AccelAngleY = [AccelAngleY; ThetaAccel];
    
    
    Phi = newAngleX;
    
    Theta = newAngleY;

end 


% 
% T = table(rad2deg(PhiAngleComplimentary),rad2deg(ThetaAngleComplimentary));

% pth = "/Users/dariusmensah/Desktop/LabTesting/MPU_6050/ComplimentaryFilter/z_AxisRotationFast.xlsx";
% 
% writetable(T,pth);
% 
% clearvars;
%clc;

heading = zeros(size(ThetaAngleComplimentary));

q = angle2quat(heading,ThetaAngleComplimentary,PhiAngleComplimentary);
q = quatnormalize(q);

[yaw,pitch,roll] = quat2angle(q);

yaw = rad2deg(yaw);
pitch = rad2deg(pitch);
roll = rad2deg(roll);

translations = zeros(size(q));
translations(:,end) = [];


figure(3)
for jj = 1:length(q)
    plotTransforms(translations(jj,:),q(jj,:), "InertialZDirection","down")
    pause(0.0005)
end
load("MPU6050_newSamples.mat")

%% Initial Values 
Phi = phi(1);
Theta = theta(1);

%% Values we want to plot 

PhiAngleComplimentary = [];
ThetaAngleComplimentary  = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.98;

dt = 1/500;

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

figure(1)
plot(time, rad2deg(AccelAngleX))
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter \Phi (Roll)")
plot(time, rad2deg(phi),'LineWidth',1)
plot(time, rad2deg(PhiAngleComplimentary),'LineWidth',1)
legend("\Phi Acclerometer", "\Phi Gyro", "\Phi Filter")
% legend("\Phi Filter","\Phi Gyro")
hold off


figure(2)
plot(time, rad2deg(AccelAngleY))
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter \Theta (Pitch)")
plot(time, rad2deg(theta),'LineWidth',1)
plot(time, rad2deg(ThetaAngleComplimentary),'LineWidth',1)
legend("\Theta Acclerometer", "\Theta Gyro", "\Theta Filter")
% legend("\Theta Filter","\Theta Gyro")
hold off


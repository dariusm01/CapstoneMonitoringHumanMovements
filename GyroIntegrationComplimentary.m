MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500; 
 
AngleSim = sim("RateGyro_to_EulerAngles.slx");

% Outputs 57x1
phi = AngleSim.phi.signals.values;
theta = AngleSim.theta.signals.values;
psi = AngleSim.psi.signals.values;
phi_dot = AngleSim.phi_dot.signals.values;
theta_dot = AngleSim.theta_dot.signals.values;
psi_dot = AngleSim.psi_dot.signals.values;

%% Resampling to get 50x1
% resamples the input sequence, x, at 7/8 times the original sample rate
% 57*(7/8) = 49.8750 -> ceil(49.8750) = 50
phi = resample(phi,7,8);
theta = resample(theta,7,8);
psi = resample(psi,7,8);
phi_dot = resample(phi_dot,7,8);
theta_dot = resample(theta_dot,7,8);
psi_dot = resample(psi_dot,7,8);

%% Initial Values 
ThetaX = phi(1);
ThetaY = theta(1);

%% Values we want to plot 

AngleXComplimentary = [];
AngleYComplimentary = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.95;

%% Complimentary Filter

for i = 1:length(time)

    % angle corrections using accelerometer 
    ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 

    ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  
    
    newAngleX = alpha*(phi_dot(i)*dt+ThetaX) + (1-alpha)*ThetaXAccel;
    
    newAngleY = alpha*(theta_dot(i)*dt+ThetaY) + (1-alpha)*ThetaYAccel;
    
    
    % Store for plotting
    AngleXComplimentary = [AngleXComplimentary;newAngleX];
    AngleYComplimentary = [AngleYComplimentary;newAngleY];
    
    AccelAngleX = [AccelAngleX; ThetaXAccel];
    AccelAngleY = [AccelAngleY; ThetaYAccel];
    
    
    ThetaX = newAngleX;
    
    ThetaY = newAngleY;

end 

figure(1)
plot(time, AngleXComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter \thetaX (Roll)")
plot(time, AccelAngleX)
plot(time, phi)
legend("\thetaX Filter", "\thetaX Acclerometer", "\thetaX Gyro")
% legend("\thetaX Filter","\thetaX Gyro")
hold off


figure(2)
plot(time, AngleYComplimentary)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter \thetaY (Pitch)")
plot(time, AccelAngleY)
plot(time, theta)
legend("\thetaY Filter", "\thetaY Acclerometer", "\thetaY Gyro")
% legend("\thetaY Filter","\thetaY Gyro")
hold off


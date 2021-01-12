MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = -1*(GyroX - mean(GyroX));   GyroY  = -1*(GyroY - mean(GyroY));   GyroZ  = -1*(GyroZ - mean(GyroZ)); 

time = MeasuredData.Time_sec;

dt = 1/500;

GyroX = deg2rad(GyroX);
GyroY = deg2rad(GyroY);
GyroZ = deg2rad(GyroZ);

%% Changing orientation to match North East Down

[AccelY,AccelX] = swap(AccelX,AccelY);

[GyroY,GyroX] = swap(GyroX,GyroY);
 
AngleSim = sim("RateGyroUsingQuaternions.slx");

% Outputs 55x1
phi = AngleSim.phi.signals.values;
theta = AngleSim.theta.signals.values;
psi = AngleSim.psi.signals.values;
phi_dot = AngleSim.phi_dot.signals.values;
theta_dot = AngleSim.theta_dot.signals.values;
psi_dot = AngleSim.psi_dot.signals.values;

%% Resampling to get 50x1
% resamples the input sequence, x, at 7/8 times the original sample rate
% 55*(9/10) = 49.50 -> ceil(49.50) = 50
phi = resample(phi,9,10);
theta = resample(theta,9,10);
psi = resample(psi,9,10);
phi_dot = resample(phi_dot,9,10);
theta_dot = resample(theta_dot,9,10);
psi_dot = resample(psi_dot,9,10);

%% Initial Values 
Phi = phi(1);
Theta = theta(1);

%% Values we want to plot 

PhiAngleComplimentary = [];
ThetaAngleComplimentary  = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.95;

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
plot(time, rad2deg(PhiAngleComplimentary))
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter \Phi (Roll)")
plot(time, rad2deg(AccelAngleX))
plot(time, rad2deg(phi))
legend("\Phi Filter", "\Phi Acclerometer", "\Phi Gyro")
% legend("\Phi Filter","\Phi Gyro")
hold off


figure(2)
plot(time, rad2deg(ThetaAngleComplimentary))
grid on
hold on
xlabel("time")
ylabel("Degrees [°]")
title("Complimentary Filter \Theta (Pitch)")
plot(time, rad2deg(AccelAngleY))
plot(time, rad2deg(theta))
legend("\Theta Filter", "\Theta Acclerometer", "\Theta Gyro")
% legend("\Theta Filter","\Theta Gyro")
hold off


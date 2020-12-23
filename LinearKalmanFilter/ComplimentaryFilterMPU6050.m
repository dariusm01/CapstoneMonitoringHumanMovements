MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;


%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = 1-(AccelZ - mean(AccelZ));

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500; 
 
% Initial Angle Values (guess)
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 


%% Values we want to plot 

AngleX = [];
AngleY = [];

AccelAngleX = [];
AccelAngleY = [];

alpha = 0.95;

for i = 1:length(time)

    % angle corrections using accelerometer 
    ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 

    ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  
    
    newAngleX = alpha*(GyroX*dt+ThetaX) + (1-alpha)*ThetaXAccel;
    
    newAngleY = alpha*(GyroY*dt+ThetaY) + (1-alpha)*ThetaYAccel;
    
    
    % Store for plotting
    AngleX = [newAngleX];
    AngleY = [newAngleY];
    
    AccelAngleX = [AccelAngleX; ThetaXAccel];
    AccelAngleY = [AccelAngleY; ThetaYAccel];
    
    
    ThetaX = newAngleX;
    
    ThetaY = newAngleY;

end 

figure(1)
plot(time, AngleX)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter \thetaX")
plot(time, AccelAngleX)
plot(time, GyroX*dt)
legend("\thetaX Filter", "\thetaX Acclerometer", "\thetaX Gyro")
% legend("\thetaX Filter","\thetaX Gyro")
hold off


figure(2)
plot(time, AngleY)
grid on
hold on
xlabel("time")
ylabel("Degrees (°)")
title("Complimentary Filter \thetaY")
plot(time, AccelAngleY)
plot(time, GyroY*dt)
legend("\thetaY Filter", "\thetaY Acclerometer", "\thetaY Gyro")
% legend("\thetaY Filter","\thetaY Gyro")
hold off


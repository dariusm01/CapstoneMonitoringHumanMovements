MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;

%% Gyro Noise Specs:
% Total RMS Noise = 0.1 °/s rms
% Rate Noise spectral density = 0.01 °/s /√Hz

%% Accelerometer Noise Specs
% Noise power spectral density (low noise mode) = 300 µg/√Hz

%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = AccelZ - mean(AccelZ);

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

time = MeasuredData.Time_sec;

dt = 1/500;

%% Prediction 

% x = Fx + w

F = [1 0 0 dt 0 0; 
     0 1 0 0 dt 0; 
     0 0 1 0 0 dt; 
     0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1];
 
Wk = 0; 
 
% Initial Angle Values - very hard to initialize 
% and estimate hidden variables 
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values - it is best to use the first measurement as the
% value for the observable variables
OmegaX = GyroX(1); OmegaY = GyroY(1); OmegaZ = GyroZ(1); 

ThetaZMag = zeros(size(GyroZ)); %no magnetometer

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

% Covariance Matrix 

Pk_1 = eye(length(F))*500;

AccelSpectralDensity = 300e-6*sqrt(dt);

GyroSpectralDensity = 0.01*sqrt(dt);

Qk = eye(length(F));

Qk(1) = AccelSpectralDensity; Qk(2,2) = Qk(1); Qk(3,3) = Qk(1);

Qk(4,4) = GyroSpectralDensity; Qk(5,5) = Qk(4,4); Qk(6,6) = Qk(4,4);

% Measurement noise
Rk = eye(size(Pk_1))*0.1;

H = eye(size(Pk_1));

I = eye(size(H));

%% Values we want to plot 

AngleXKalman = [];
AngleYKalman = [];
AngleZKalman = [];
OmegaXKalman = [];
OmegaYKalman = [];
OmegaZKalman = [];

AngleXAccelerometer = [];
AngleYAccelerometer = [];

%% Need the standard deviation and residuals to test evaluate the filter mathematically
PosThetaXSTD = [];
PosThetaYSTD = [];

SpeedThetaXSTD = [];
SpeedThetaYSTD = [];

ResidualThetaX = [];
ResidualThetaY = [];

ResidualOmegaX = [];
ResidualOmegaY = [];

for i = 1:length(time)

    % Prior
    Xkp = F*Xk_1 + Wk;  

    % Proccess Covariance matrix
    Pkp = F*Pk_1*F.'+ Qk;  
    
    % Innovation Covariance
    Sk = H*Pk_1*H.' + Rk;
    
    % Measurement (evidence)
    
        % angle corrections using accelerometer 
        ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 

        ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  
    
    zk = [ThetaXAccel; ThetaYAccel; ThetaZMag(i); GyroX(i); GyroY(i); GyroZ(i)];  
    
    % Innovation (Residual)
    yk = zk - H*Xkp;
    
    % Kalman Gain  
    K = Pkp*H.'*(Sk^-1);
    
    % Posterior 
    Xk = Xkp + K*yk;
    
    % Covariance Update
    Pk = (I - K*H)*Pkp*(I - K*H).' + (K*Rk*K.');
    
    % Redefining for next iteration
    Xk_1 = Xk;
    
    Pk_1 = Pk;
    
    
    % Store for plotting
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
    AngleZKalman = [AngleZKalman; Xk(3)];
    OmegaXKalman = [OmegaXKalman; Xk(4)];
    OmegaYKalman = [OmegaYKalman; Xk(5)];
    OmegaZKalman = [OmegaZKalman; Xk(6)];
    
    AngleXAccelerometer = [AngleXAccelerometer; zk(1)];
    AngleYAccelerometer = [AngleYAccelerometer; zk(2)];
    
    PosThetaXSTD = [PosThetaXSTD; sqrt(Pk(1,1))];
    PosThetaYSTD = [PosThetaYSTD; sqrt(Pk(2,2))];
    SpeedThetaXSTD = [SpeedThetaXSTD; sqrt(Pk(4,4))];
    SpeedThetaYSTD = [SpeedThetaYSTD; sqrt(Pk(5,5))];

    ResidualThetaX = [ResidualThetaX; yk(1)];
    ResidualThetaY = [ResidualThetaY; yk(2)];
    ResidualOmegaX = [ResidualOmegaX; yk(4)];
    ResidualOmegaY = [ResidualOmegaY; yk(5)];
end 


%% Plotting

% figure(1)
% plot(time, GyroX)
% title("Gyroscope X Axis");
% xlabel("Time(s)")
% ylabel("Degrees/sec")
% grid on
% hold on 
% plot(time, OmegaXKalman)
% legend("Measured Gyro data (X direction)", "Kalman Filter Gyro data (X direction)")
% hold off
% 
% figure(2)
% plot(time, GyroY)
% title("Gyroscope Y Axis");
% xlabel("Time(s)")
% ylabel("Degrees/sec")
% grid on
% hold on 
% plot(time, OmegaYKalman)
% legend("Measured Gyro data (Y direction)", "Kalman Filter Gyro data (Y direction)")
% hold off
% 
% figure(3)
% plot(time, GyroZ)
% title("Gyroscope Z Axis");
% xlabel("Time(s)")
% ylabel("Degrees/sec")
% grid on
% hold on 
% plot(time, OmegaZKalman)
% legend("Measured Gyro data (Z direction)", "Kalman Filter Gyro data (Z direction)")
% hold off
% 
% figure(4)
% plot(time, AngleXAccelerometer)
% title("Angle X Axis");
% xlabel("Time(s)")
% ylabel("Degrees")
% grid on
% hold on 
% plot(time, AngleXKalman)
% legend("Accelerometer Angle Correction (X)", "Kalman Filter Angle (X direction)")
% hold off
% 
% figure(5)
% plot(time, AngleYAccelerometer)
% title("Angle Y Axis");
% xlabel("Time(s)")
% ylabel("Degrees")
% grid on
% hold on 
% plot(time, AngleYKalman)
% legend("Accelerometer Angle Correction (Y)","Kalman Filter Angle (Y direction)")
% hold off


figure(6)
plot(time, 3*PosThetaXSTD, 'o')
title("Angular Position Residuals ThetaX")
ylabel("Degrees")
grid on
hold on
% plot(time, ResidualThetaX)
plot(time, -3*PosThetaXSTD, 'o')
hold off

figure(7)
plot(time, PosThetaYSTD, 'o')
title("Angular Position Residuals ThetaY")
ylabel("Degrees")
grid on
hold on
% plot(time, ResidualThetaY)
plot(time, -1*PosThetaYSTD, 'o')
hold off

figure(8)
plot(time, 5*SpeedThetaXSTD, 'ko')
title("\omega X Residuals 5\sigma")
ylabel("Degrees/sec")
grid on
hold on
plot(time, ResidualOmegaX)
plot(time, -5*SpeedThetaXSTD, 'ko')
hold off

figure(9)
plot(time, 5*SpeedThetaYSTD, 'ko')
title("\omega Y Residuals 5\sigma")
ylabel("Degrees/sec")
grid on
hold on
plot(time, ResidualOmegaY)
plot(time, -5*SpeedThetaYSTD, 'ko')
hold off


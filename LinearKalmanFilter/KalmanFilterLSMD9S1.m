%% Gathering Sensor Data
GyroData = readtable("/Users/dariusmensah/Desktop/IMUData/1_ESP32_781Gyro.xlsx");
AccelData = readtable("/Users/dariusmensah/Desktop/IMUData/1_ESP32_781Acc.xlsx");
MagData = readtable("/Users/dariusmensah/Desktop/IMUData/1_ESP32_781Mag.xlsx"); 

%% Accelerometer using the LSM9DS1 +/- 2g
% Outputs in micro g
% 0.061 = sensitivity factor
AccelX = (AccelData.AccX*0.061)/1e3;
AccelY = (AccelData.AccY*0.061)/1e3;
AccelZ = (AccelData.AccZ*0.061)/1e3;   %Says Gravity is downwards (negative)
Time_Accel = (AccelData.Time_milliseconds)/1e3;
Accel_dt = ((Time_Accel(2)-Time_Accel(1))+(Time_Accel(3)-Time_Accel(2)))/2;

%% Gyroscope using the LSM9DS1 +/- 245 deg/sec
% Outputs in micro deg/sec
% 8.75 = sensitivity factor
GyroX = (GyroData.GyroX/8.75)/1e3;
GyroY = (GyroData.GyroY/8.75)/1e3;
GyroZ = (GyroData.GyroZ/8.75)/1e3;
Time_Gyro = (GyroData.Time_milliseconds)/1e3;
Gyro_dt = ((Time_Gyro(2)-Time_Gyro(1))+(Time_Gyro(3)-Time_Gyro(2)))/2;

%% Magnetometer using the LSM9DS1 +/- 4 Gauss 
% Outputs in micro Gauss
% 0.14 = sensitivity factor
Mag_X = (MagData.MagX*0.14)/1e3;
Mag_Y = (MagData.MagY*0.14)/1e3;
Mag_Z = (MagData.MagZ*0.14)/1e3;
Time_Mag = (MagData.Time_milliseconds)/1e3;
Mag_dt = ((Time_Mag(2)-Time_Mag(1))+(Time_Mag(3)-Time_Mag(2)))/2;

%% Calibrated Magnetometer Data
[MagX, MagY, MagZ] = MagnetometerCalibration(Mag_X, Mag_Y, Mag_Z);

% Magnetic Inclinaton Middletown, PA
Inclination = 66.27513; % degrees
I = Inclination*(pi/180); % radians 
% ([µT] = 0.01 Gauss [G])

%% dt 
dt = (Accel_dt + Gyro_dt + Mag_dt)/3;

%% Linear Discrete Time Kalman Filter States & Estimates

F = [1 0 0 dt 0 0; 
     0 1 0 0 dt 0; 
     0 0 1 0 0 dt; 
     0 0 0 1 0 0; 
     0 0 0 0 1 0; 
     0 0 0 0 0 1];

ExampleNoise = wgn(3,1,0);

Ex =  ExampleNoise(1); Ey = ExampleNoise(2) ; Ez = ExampleNoise(3); % random noise
 
Uk = [Ex; Ey; Ez]*(1/10e5);
% Uk = [0;  0;  0];

G = [0.5*dt^2 0 0;
     0 0.5*dt^2 0;
     0 0 0.5*dt^2;
     dt 0 0;
     0 dt 0;
     0 0 dt];
 
noise = wgn(6,1,0);

% Noise terms for dead-reckoning
Xix = noise(1); Xiy = noise(2); Xiz = noise(3); 

% Noise terms for gyro
Nx = noise(4); Ny = noise(5); Nz = noise(6); 

% Column Matrix of Noise
% Wk = [Xix; Xiy; Xiz; Nx; Ny; Nz];
% Wk = abs(Wk);
Wk = 0;

% Initial Angle Values
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values (estimations not sensor)
OmegaX = 0; OmegaY = 0; OmegaZ = 0; 

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

 %% Linear Discrete Time Kalman Filter Uncertainty in the Estimates
 
Pk_1 = eye(length(F)); 
% A = [AngleXAccelerometer AngleYAccelerometer AngleZMagnetometer GyroX GyroY GyroZ];
% Unity = ones(length(A));
% a = A - ((Unity*A)*(1/length(A)));
% Covariance = a.'*a;
% diag(Covariance)
Pk_1(1) = 904.0528; Pk_1(2,2) = 175.2210; Pk_1(3,3) = 8.8808; 
Pk_1(4,4) = 35.1119; Pk_1(5,5) = 5.4441; Pk_1(6,6) = 28.1223;

% Process Noise
ProcessNoise = abs(wgn(6,1,0));     
Qk = eye(length(F));
Qk(1) = std(Uk)*ProcessNoise(1); % essentially the std(acceleration [epsilon]) 
Qk(2,2) = std(Uk)*ProcessNoise(2); 
Qk(3,3) = std(Uk)*ProcessNoise(3); 
Qk(4,4) = std(Uk)*ProcessNoise(4); 
Qk(5,5) = std(Uk)*ProcessNoise(5); 
Qk(6,6) = std(Uk)*ProcessNoise(6); 
Qk = abs(Qk);

H = eye(length(F));

% Measurment Noise
MeasurmentNoise = wgn(6,1,0); 

%% Values we want to plot 

AngleXKalman = [];
AngleYKalman = [];
AngleZKalman = [];
OmegaXKalman = [];
OmegaYKalman = [];
OmegaZKalman = [];

AngleXAccelerometer = [];
AngleYAccelerometer = [];
AngleZMagnetometer = [];

%% Filtering the data

for i = 1:length(Time_Gyro)
    
     % Prediction
     Xkp = F*Xk_1 + G*Uk + Wk;    
     
     % Covariance matrix
     Pkp = F*Pk_1*F.'+ Qk;      

     P = diag(Pkp);
     
     Pkp = eye(length(F));
     
     Pkp(1) = P(1); Pkp(2,2) = P(2); Pkp(3,3) = P(3); Pkp(4,4) = P(4); Pkp(5,5) = P(5); Pkp(6,6) = P(6); 
    
     % Trying a more precise approach using measurment standard deviation
     R = eye(size(Pkp));
     R(1) = (0.2283)*MeasurmentNoise(1);      % std(AngleXAccelerometer)
     R(2,2) = (0.1005)*MeasurmentNoise(2);    % std(AngleYAccelerometer)
     R(3,3) = (0.0226)*MeasurmentNoise(3);    % std(AngleZMagnetometer)
     R(4,4) = (0.0450)*MeasurmentNoise(4);    % std(GyroX)
     R(5,5) = (0.0177)*MeasurmentNoise(5);    % std(GyroY)
     R(6,6) = ( 0.0403)*MeasurmentNoise(6);   % std(GyroZ)
     R = abs(R);      

     % Kalman Gain
     k = Pkp*H.'*((H*Pkp*H.' + R)^(-1)); 
     
     K = diag(k);
     
     k = eye(length(F));
     
     k(1) = K(1); k(2,2) = K(2); k(3,3) = K(3); k(4,4) = K(4); k(5,5) = K(5); k(6,6) = K(6);
     
     k = abs(k);
     
     % angle corrections using accelerometer 
     ThetaXAccel = atan2(-AccelY(i),AccelZ(i)); 
     
     ThetaYAccel = atan2(AccelX(i),sqrt((AccelY(i)^2) + (AccelZ(i)^2)));
     
     % angle corrections using magnetometer
     M = sqrt((MagX(i)^2) + (MagY(i)^2) + (MagX(i)^2)); % Field Magnitude
     
     Num = cos(ThetaYAccel)*((MagZ(i)*sin(ThetaXAccel))-(MagY(i)*cos(ThetaXAccel)));
     Den = MagX(i)+(M*(sin(I))*sin(ThetaYAccel));
     ThetaZMag = atan2(Num,Den);
     
      %ThetaZMag = atan2((-MagY(i)*cos(ThetaXAccel) + MagZ(i)*sin(ThetaXAccel) ) , (MagX(i)*cos(ThetaYAccel) + MagY(i)*sin(ThetaYAccel)*sin(ThetaXAccel)+ MagZ(i)*sin(ThetaYAccel)*cos(ThetaXAccel))); 
      %ThetaZMag = atan2((MagX(i)),(MagY(i)));
      
     
     % Measurements 
     Ykm = [ThetaXAccel; ThetaYAccel; ThetaZMag; GyroX(i); GyroY(i); GyroZ(i)];  
    

     % Measurement uncertainty
     zk = ones(size(Ykm)); 

     zk(1) = -2.8840;    % -mean(AngleXAccelerometer)
     zk(2) = 0.1233;     % -mean(AngleYAccelerometer)
     zk(3) = -1.6035;    % -mean(AngleZMagnetometer)
     zk(4) = -0.0011;    % -mean(GyroX)
     zk(5) = 0.1299;     % -mean(GyroY)
     zk(6) = -1.8809;    % -mean(GyroZ)

     C = eye(length(F));
     
     %Angle corrections are in radians, converting to degrees
     C(1) = 180/pi; C(2,2) = C(1); C(3,3) = C(2,2);

     Yk = C*Ykm + zk;

     % Updating the estimation
     Xk = Xkp +  k*(Ykm - H*Xkp);   

     % Updating the uncertainty
     Pk = (eye(size(H)) - k*H)*Pkp; 
     
    % redefining for the next iterations
     Xk_1 = Xk;
     
     Pk_1 = Pk;
     
    % store for plotting
    AngleXKalman = [AngleXKalman; Xk(1)];
    AngleYKalman = [AngleYKalman; Xk(2)];
    AngleZKalman = [AngleZKalman; Xk(3)];
    OmegaXKalman = [OmegaXKalman; Xk(4)];
    OmegaYKalman = [OmegaYKalman; Xk(5)];
    OmegaZKalman = [OmegaZKalman; Xk(6)];
    
    AngleXAccelerometer = [AngleXAccelerometer; Ykm(1)];
    AngleYAccelerometer = [AngleYAccelerometer; Ykm(2)];
    AngleZMagnetometer = [AngleZMagnetometer; Ykm(3)];

end 


%% Plotting

figure(1)
plot(Time_Gyro, GyroX)
title("Gyroscope X Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(Time_Gyro, OmegaXKalman)
legend("Measured Gyro data (X direction)", "Kalman Filter Gyro data (X direction)")
hold off

figure(2)
plot(Time_Gyro, GyroY)
title("Gyroscope Y Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(Time_Gyro, OmegaYKalman)
legend("Measured Gyro data (Y direction)", "Kalman Filter Gyro data (Y direction)")
hold off

figure(3)
plot(Time_Gyro, GyroZ)
title("Gyroscope Z Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(Time_Gyro, OmegaZKalman)
legend("Measured Gyro data (Z direction)", "Kalman Filter Gyro data (Z direction)")
hold off

figure(4)
plot(Time_Accel, AngleXAccelerometer)
title("Angle X Axis");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(Time_Accel, AngleXKalman)
legend("Accelerometer Angle Correction (X)", "Kalman Filter Angle (X direction)")
hold off

figure(5)
plot(Time_Accel, AngleYAccelerometer)
title("Angle Y Axis");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(Time_Accel, AngleYKalman)
legend("Accelerometer Angle Correction (Y)","Kalman Filter Angle (Y direction)")
hold off

figure(6)
plot(Time_Mag, AngleZMagnetometer)
title("Angle Z Axis");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(Time_Mag, AngleZKalman)
legend("Magnetometer Angle Correction", "Kalman Filter Angle (Z direction)")
hold off

%% Clear everything

% clearvars; close all; clc
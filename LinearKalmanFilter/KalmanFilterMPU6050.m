%% Simplifications & Explanations:
%
%   - The covariance among the variables is close to zero
%   - Initial covariance matrix is an identity 
%   - The time step is generic
%   - White Gaussian noise is the only random noise so far
%   - Ignoring the off diagonal values for Kalman Gain
%   - Due to the noise in the system, each iteration will produce different
%     results
%   - I need to tune: 
%       ~ Qk (the process noise covariance matrix) 
%       ~ Pk_1 the initial error covariance.
%       ~ and the measurement noise covariance R

%% Measured Data

MeasuredData = readtable("/Users/dariusmensah/Desktop/SampleData.xlsx");    % change to your specific file path

AccelX = MeasuredData.AcX/16384;  AccelY = MeasuredData.AcY/16384;  AccelZ = MeasuredData.AcZ/16384;
% Dividing by 16384 to get +/- 2g via data sheet
% Accelerometer Sample Rate = 1 kHz

GyroX = MeasuredData.GyX/131;   GyroY = MeasuredData.GyY/131;   GyroZ = MeasuredData.GyZ/131;
 
% Dividing by 131 to get +/- 250deg/sec via data sheet
% Gyroscope Sample Rate, Fast = 8 kHz
% Gyroscope Sample Rate, Slow = 1 kHz

%% Simple form of calibration by removing the mean values
AccelX = AccelX - mean(AccelX); AccelY = AccelY - mean(AccelY); AccelZ = AccelZ - mean(AccelZ);

GyroX  = GyroX - mean(GyroX);   GyroY  = GyroY - mean(GyroY);   GyroZ  = GyroZ - mean(GyroZ); 

% Data sheet says I^2C Operating Frequency all registers, Standard-mode =
% 100 kHz

time = MeasuredData.Time_sec;

% dt = 1/2875;    % = avg(2.61kHz, 2.96kHz)

dt = 1/500; % Generic 500Hz

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
% Wk = [Xix; Xiy; Xiz; Nx; Ny; Nz]*(1/10e3);
% Wk = abs(Wk);
Wk = 0;

% Initial Angle Values
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values (estimations not sensor)
OmegaX = 0; OmegaY = 0; OmegaZ = 0; 

ThetaZMag = zeros(size(GyroZ)); %no magnetometer

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

 %% Linear Discrete Time Kalman Filter Uncertainty in the Estimates
 
Pk_1 = eye(length(F)); 
% A = [AngleXAccelerometer AngleYAccelerometer AngleZMagnetometer GyroX GyroY GyroZ];
% Unity = ones(length(A));
% a = A - ((Unity*A)*(1/length(A)));
% Covariance = a.'*a;
% diag(Covariance)
Pk_1(1) = 0.7913; Pk_1(2,2) = 1.8798; 
Pk_1(4,4) = 0.5105; Pk_1(5,5) = 0.7494; Pk_1(6,6) = 0.4919;

% Process Noise
ProcessNoise =wgn(6,1,0);     
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
%% Filtering the data

for i = 1:length(time)
    
     % Prediction
     Xkp = F*Xk_1 + G*Uk + Wk;    
     
     % Covariance matrix
     Pkp = F*Pk_1*F.'+ Qk;      

     P = diag(Pkp);
     
     Pkp = eye(length(F));
     
     Pkp(1) = P(1); Pkp(2,2) = P(2); Pkp(3,3) = P(3); Pkp(4,4) = P(4); Pkp(5,5) = P(5); Pkp(6,6) = P(6); 
    
     % Trying a more precise approach using measurment standard deviation
     R = eye(size(Pkp));
     R(1) = (0.0022)*MeasurmentNoise(1);      % std(AngleXAccelerometer)
     R(2,2) = (0.0034)*MeasurmentNoise(2);    % std(AngleYAccelerometer)
     R(3,3) = (0.05)*MeasurmentNoise(3);      % generic
     R(4,4) = (0.1021)*MeasurmentNoise(4);    % std(GyroX)
     R(5,5) = (0.1237)*MeasurmentNoise(5);    % std(GyroY)
     R(6,6) = (0.1002)*MeasurmentNoise(6);    % std(GyroZ)
     R = abs(R);      

     % Kalman Gain
     k = Pkp*H.'*((H*Pkp*H.' + R)^(-1)); 
     
     K = diag(k);
     
     k = eye(length(F));
     
     k(1) = K(1); k(2,2) = K(2); k(3,3) = K(3); k(4,4) = K(4); k(5,5) = K(5); k(6,6) = K(6);
     
     k = abs(k);
     
     % angle corrections using accelerometer 
     ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 
     
     ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  

     % Measurements 
     Ykm = [ThetaXAccel; ThetaYAccel; ThetaZMag(i); GyroX(i); GyroY(i); GyroZ(i)];  
    

     % Measurement uncertainty
     zk = ones(size(Ykm)); 

     zk(1) = -0.0019;    % -mean(AngleXAccelerometer)
     zk(2) = 0.0324;     % -mean(AngleYAccelerometer)
     zk(3) = -0.025;     %  generic
     zk(4) = -0.2937;    % -mean(GyroX)
     zk(5) = -2.5273;    % -mean(GyroY)
     zk(6) = 0.2211;     % -mean(GyroZ)

     C = eye(length(F));

     Yk = C*Ykm + zk;

     % Updating the estimation
     % The innovation is a measurement residual (observed - mean)
     % Adding the residual to the current estimation gives the new
     % information about the state
     
     Innovation = (Ykm - H*Xkp);
     Xk = Xkp +  k*Innovation;

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


end 

%% Plotting

figure(1)
plot(time, GyroX)
title("Gyroscope X Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaXKalman)
legend("Measured Gyro data (X direction)", "Kalman Filter Gyro data (X direction)")
hold off

figure(2)
plot(time, GyroY)
title("Gyroscope Y Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaYKalman)
legend("Measured Gyro data (Y direction)", "Kalman Filter Gyro data (Y direction)")
hold off

figure(3)
plot(time, GyroZ)
title("Gyroscope Z Axis");
xlabel("Time(s)")
ylabel("Degrees/sec")
grid on
hold on 
plot(time, OmegaZKalman)
legend("Measured Gyro data (Z direction)", "Kalman Filter Gyro data (Z direction)")
hold off

figure(4)
plot(time, AngleXAccelerometer)
title("Angle X Axis");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, AngleXKalman)
legend("Accelerometer Angle Correction (X)", "Kalman Filter Angle (X direction)")
hold off

figure(5)
plot(time, AngleYAccelerometer)
title("Angle Y Axis");
xlabel("Time(s)")
ylabel("Degrees")
grid on
hold on 
plot(time, AngleYKalman)
legend("Accelerometer Angle Correction (Y)","Kalman Filter Angle (Y direction)")
hold off

% figure(6)
% plot(time, 0)
% refline(0, 0);
% title("Angle Z Axis");
% xlabel("Time(s)")
% ylabel("Degrees")
% grid on
% hold on 
% plot(time, AngleZKalman)
% legend("Approximate Angle (Z direction)", "Kalman Filter Angle (Z direction)")
% hold off

%% Clear everything

% clearvars; close all; clc
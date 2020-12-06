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

%% To find the means to get a good corvariance matrix
AngleX = [];
AngleY = [];
AngleZ = [];
RateX = [];
RateY = [];
RateZ = [];

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
Wk = [Xix; Xiy; Xiz; Nx; Ny; Nz]*(1/10e3);

% Initial Angle Values
ThetaX = 0; ThetaY = 0; ThetaZ = 0; 

% Initial Gyro Values (estimations not sensor)
OmegaX = 0; OmegaY = 0; OmegaZ = 0; 

ThetaZMag = zeros(size(GyroZ)); %no magnetometer

% State Matrix
Xk_1 = [ThetaX; ThetaY; ThetaZ; OmegaX; OmegaY; OmegaZ]; 

 %% Linear Discrete Time Kalman Filter Uncertainty in the Estimates
 
% Pk_1 = eye(length(F)); 
% % A = [AngleXAccelerometer AngleYAccelerometer AngleZKalman GyroX GyroY GyroZ];
% % Unity = ones(length(A));
% % a = A - ((Unity*A)*(1/length(A)));
% % Covariance = a.'*a;
% % diag(Covariance)
% Pk_1(1) = 3.4626e4; Pk_1(2,2) = 7.9895e4; 
% Pk_1(4,4) = 0.5105; Pk_1(5,5) = 0.0001e4; Pk_1(6,6) = 0.0001e4;

% Process Noise
ProcessNoise =wgn(6,1,0);
dimension = ones(1,length(F));

% essentially the variance(acceleration [epsilon])
Qk_old = var(Uk);

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
     Qk = Qk_old*(G*G.');
     
     %% Covariance Matrix
     % The 'expectation' is denoted by capital letter E.
	
	 % The expectation of the random variable E(X) equals to 
	 % the mean of the random variable
    
     % E(X)=μX
 
     % where μX is the mean of the random variable.
     AngleX = [AngleX; Xkp(1)];
     AngleY = [AngleY; Xkp(2)];
     AngleZ = [AngleZ; Xkp(3)];
     RateX = [RateX; Xkp(4)];
     RateY = [RateY; Xkp(5)];
     RateZ = [RateZ; Xkp(6)];
     
     mu = [mean(AngleX);mean(AngleY);mean(AngleZ);mean(RateX);mean(RateY);mean(RateZ)];
     
     % Pn,n=E((x̂ n,n−μxn,n)(x̂ n,n−μxn,n)T)
%      Estate1  = mean(Xkp(1) - mu(1));
%      Estate2  = mean(Xkp(2) - mu(2));
%      Estate3  = mean(Xkp(3) - mu(3));
%      Estate4  = mean(Xkp(4) - mu(4));
%      Estate5  = mean(Xkp(5) - mu(5));
%      Estate6  = mean(Xkp(6) - mu(6));
%      
%      Estates = [Estate1; Estate2; Estate3; Estate4; Estate5; Estate6];
%      
%      Pk_1 = Estates*Estates.';
     Pk_1 = (Xkp-mu)*(Xkp-mu).';
     
     Pkp = F*Pk_1*F.'+ Qk;  
     
     P = diag(Pkp);
     
     Pkp = eye(length(F));
     
     Pkp(1) = P(1); Pkp(2,2) = P(2); Pkp(3,3) = P(3); Pkp(4,4) = P(4); Pkp(5,5) = P(5); Pkp(6,6) = P(6); 
    
     
     Qk_old = Qk;
     
     % angle corrections using accelerometer 
     ThetaXAccel = (atan2(AccelY(i), sqrt((AccelX(i)^2) + (AccelZ(i)^2)))) * (180/pi); 
     
     ThetaYAccel = atan2(-AccelX(i), sqrt((AccelY(i)^2) + (AccelZ(i)^2))) * (180/pi);  

     % Measurements 
     Ykm = [ThetaXAccel; ThetaYAccel; ThetaZMag(i); GyroX(i); GyroY(i); GyroZ(i)];  
    
     
     % Measurement Noise covariance
     v = [var(ThetaXAccel); var(ThetaYAccel); var(ThetaZMag); var(GyroX);
         var(GyroY); var(GyroZ)];
     
     R = v*v.';
     
     % Kalman Gain
     k = Pkp*H.'*((H*Pkp*H.' + R)^(-1)); 

     % Measurement uncertainty
     zk = ones(size(Ykm)); 

     zk(1) = -1.0275;     % -mean(AngleXAccelerometer)
     zk(2) = 3.2857;      % -mean(AngleYAccelerometer)
     zk(3) = -0.025;      %  generic
     zk(4) = 3.3862e-17;  % -mean(GyroX)
     zk(5) = -1.1546e-16; % -mean(GyroY)
     zk(6) = -2.4425e-17; % -mean(GyroZ)

     C = eye(length(F));

     Yk = C*Ykm + zk;

     % Updating the estimation
     % The innovation is a measurement residual (observed - mean)
     % Adding the residual to the current estimation gives the new
     % information about the state
     
     Innovation = (Ykm - H*Xkp);
     Xk = Xkp +  k*Innovation;

     % Updating the uncertainty
     Pk = (eye(size(H)) - k*H)*Pkp*(eye(size(H)) - k*H).' + (k*R*k.'); 
     
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